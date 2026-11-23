#!/usr/bin/env python3
"""mutation.py tests the tests.

It makes small changes (mutants) to the safety C code and runs the test suite against each one:

  opendbc/safety/modes/byd.h
    original:    safety.gas_pressed = (msg->data[3] & 0x7FU) > 0U;
    comparison:  safety.gas_pressed = (msg->data[3] & 0x7FU) <= 0U;   <- gas reads as pressed exactly when it isn't
    boundary:    safety.gas_pressed = (msg->data[3] & 0x7FU) > 1U;    <- a throttle of exactly 1 reads as released
    deletion:    safety.gas_pressed = (msg->data[3]) > 0U;            <- the mask is gone: other bits read as gas

Deletions remove an operation entirely: a statement, a branch's body, `return;`, the operator of a compound
assignment, or an operand of a logical, bitwise or shift operator (the arithmetic mutants already show an arithmetic
operation isn't a no-op). A surviving deletion is a missing test or dead code (MISRA C:2012 Rule 2.2: an operation
whose removal doesn't change behavior).

A mutant is killed when a test fails on it: the tests would catch that bug. A surviving mutant means no test checks
that behavior, even if a test runs the line. The goal is to kill every mutant.

Usage: python -m opendbc.safety.tests.mutation [-j JOBS] [--only ID ...] [--list] [--known]
Survivors are printed with their id and source line. After adding a test, rerun just those with --only.
"""
import os
# Forking a process with live BLAS threads (numpy, imported by the tests) can deadlock.
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
  os.environ.setdefault(_v, "1")

import argparse
import ctypes
import gc
import itertools
import multiprocessing
import operator
import pickle
import random
import re
import resource
import select
import signal
import subprocess
import sys
import tempfile
import time
import traceback
import unittest
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

import tree_sitter_c as ts_c
import tree_sitter as ts

from opendbc.safety.tests.libsafety import libsafety_py

ROOT = Path(__file__).resolve().parents[3]
SAFETY_DIR = ROOT / "opendbc/safety"
TESTS_DIR = SAFETY_DIR / "tests"
# these build their own library from the unmutated sources, so they can't observe a mutant
EXCLUDED_TESTS = {"test_globals.py", "test_release_build.py"}
# mutants known to survive, each a missing test or dead code to fix; not run unless asked for (they'd run every test)
KNOWN_SURVIVORS = TESTS_DIR / "mutation_known_survivors.txt"

# As libsafety_py builds it, plus bounds checks on trailing arrays (e.g. CANPacket_t.data) and a fixed pattern in
# uninitialized locals (a deleted assignment can leave one): reading either would otherwise return whatever is in
# memory, making the verdict nondeterministic.
CFLAGS = ["-fPIC", "-shared", "-w", "-fno-builtin", "-std=gnu11", "-g0", "-O0", "-DALLOW_DEBUG", "-fsanitize=undefined",
          "-fno-sanitize-recover=undefined", "-fstrict-flex-arrays=3", "-ftrivial-auto-var-init=pattern",
          *libsafety_py.single_precision_flags()]

COMPARISONS = {"==": "!=", "!=": "==", ">": "<=", ">=": "<", "<": ">=", "<=": ">"}
RULES = {  # (tree-sitter node type, operator) -> (mutator, mutated operator)
  ("update_expression", "++"): ("increment", "--"),
  ("update_expression", "--"): ("decrement", "++"),
  ("unary_expression", "!"): ("remove_negation", ""),
  **{("binary_expression", a): ("comparison", b) for a, b in COMPARISONS.items()},
  **{("binary_expression", a): ("bitwise", b) for a, b in {"&": "|", "|": "&", "^": "&"}.items()},
  **{("binary_expression", a): ("arithmetic", b) for a, b in {"+": "-", "-": "+", "*": "/", "/": "*", "%": "*"}.items()},
  **{("assignment_expression", a): ("bitwise_assignment", b) for a, b in {"&=": "|=", "|=": "&=", "^=": "&="}.items()},
  **{("assignment_expression", a): ("arithmetic_assignment", b)
     for a, b in {"+=": "-=", "-=": "+=", "*=": "/=", "/=": "*=", "%=": "*="}.items()},
}
# compound assignments, deleted: x |= y -> x = y
COMPOUND_ASSIGNMENTS = {"+=", "-=", "*=", "/=", "%=", "&=", "|=", "^=", "<<=", ">>="}
# binary operators whose left (L) or right (R) operand can be deleted, leaving the other: x & m -> x, a && b -> b
DELETE_OPERAND = {**dict.fromkeys(("&&", "||", "&", "|", "^"), "LR"), **dict.fromkeys(("<<", ">>"), "R")}
# statement mutators: the statement (a branch's body, for delete_branch) is skipped instead of changing an expression
STATEMENT_MUTATORS = {"delete_statement", "delete_return", "delete_branch"}
# where a statement can be wrapped in braces
STATEMENT_PARENTS = {"compound_statement", "case_statement", "labeled_statement", "if_statement", "else_clause",
                     "while_statement", "for_statement", "do_statement"}
# operands of these are never evaluated
UNEVALUATED = {"sizeof_expression", "alignof_expression", "offsetof_expression"}
# on operands and results in [0, 2**31), these compute exactly what C does, for int and unsigned operands alike
FOLD_OPS = {"+": operator.add, "-": operator.sub, "*": operator.mul, "/": operator.floordiv, "%": operator.mod,
            "&": operator.and_, "|": operator.or_, "^": operator.xor, "<<": operator.lshift, ">>": operator.rshift,
            "==": operator.eq, "!=": operator.ne, "<": operator.lt, "<=": operator.le, ">": operator.gt, ">=": operator.ge}
POSITIVE = "positive"  # what _fold makes of a sizeof
INT_CASTS = {"int", "unsigned", "unsigned int", "int32_t", "uint32_t"}  # casts that keep every value in [0, 2**31)

KILLED, SURVIVED, UNCOVERED, INFRA = "killed", "survived", "uncovered", "infra_error"
KNOWN, UNKNOWN = "known_survivor", "killed_known_survivor"  # a known survivor that survives, or is killed
UBSAN_EXIT_CODE = 86  # a child stopped by UBSan exits with this (see PRELUDE), which nothing else in it uses
# signals the code running in a child can raise itself; any other signal comes from outside
CRASH_SIGNALS = {signal.SIGSEGV, signal.SIGBUS, signal.SIGFPE, signal.SIGILL, signal.SIGABRT, signal.SIGTRAP}
CPU_LIMIT_FIXTURE = 1.0  # CPU seconds a child may spend outside of tests at a time
MEM_HEADROOM = 256 << 20  # address space a child may allocate beyond what it inherits (Linux only)
RED, GREEN, YELLOW, BOLD = "31", "32", "33", "1"


def colorize(text, color):
  if not sys.stdout.isatty() or os.environ.get("TERM", "") in ("", "dumb") or "NO_COLOR" in os.environ:
    return str(text)
  return f"\033[{color}m{text}\033[0m"


@dataclass(frozen=True)
class Site:
  id: int
  start: int  # the mutated expression
  end: int
  op_start: int  # the replaced token
  op_end: int
  original: str
  mutated: str
  mutator: str
  file: Path
  line: int
  # in a static object's initializer: (declaration start, end, name start, end, end of the object's scope)
  static: tuple | None = None
  function: str = ""  # the function (or object) it's in

  def change(self, width=None):
    """The mutation, as `original->mutated` or, for a deletion, `-deleted` (without line markers, at most `width` long)."""
    if not (self.mutator.startswith("delete") and not self.mutated):
      return f"{self.original}->{self.mutated}"
    deleted = " ".join(" ".join(ln for ln in self.original.splitlines() if not ln.startswith("#")).split())
    return f"-{deleted if width is None or len(deleted) <= width else deleted[:width - 3] + '...'}"

  def __str__(self):
    return f"#{self.id:04d} {self.file.relative_to(ROOT)}:{self.line} [{self.mutator}] {colorize(self.change(60), RED)}"


# ---------------------------------------------------------------- mutation sites

def _boundary(token):
  """An integer literal (tree-sitter includes an attached sign) plus one, keeping base and suffix; None for others."""
  m = re.fullmatch(r"([-+]?)(0[xX][0-9a-fA-F]+|0|[1-9][0-9]*)([uUlL]*)", token)
  if m is None:
    return None
  value = int(m[1] + m[2], 0) + 1
  digits = f"0x{abs(value):X}" if m[2][1:2] in ("x", "X") else str(abs(value))
  return ("-" if value < 0 else "") + digits + m[3]


def _fold(txt, node, sub, picks):
  """Value of `node` if it's a constant expression of int or unsigned int literals with every value in [0, 2**31),
  None otherwise. A sizeof is an unknown positive value, POSITIVE. `sub` replaces the token starting at an offset, or
  a binary expression at (start, end) offsets by its "left" or "right" operand, and `picks` the branch taken by a
  conditional (at an offset) whose condition isn't constant; without one, it folds if both branches fold to the same
  value."""
  def text(n):
    return sub.get(n.start_byte, txt[n.start_byte:n.end_byte])
  if node.type == "parenthesized_expression" and node.named_child_count == 1:
    return _fold(txt, node.named_children[0], sub, picks)
  if (kept := sub.get((node.start_byte, node.end_byte))) is not None:
    return _fold(txt, node.child_by_field_name(kept), sub, picks)
  if node.type == "sizeof_expression":
    return POSITIVE
  if node.type == "cast_expression" and text(node.child_by_field_name("type")) in INT_CASTS:
    return _fold(txt, node.child_by_field_name("value"), sub, picks)
  if node.type == "conditional_expression":
    c = _fold(txt, node.child_by_field_name("condition"), sub, picks)
    c = picks.get(node.start_byte) if c is None else c
    a, b = (_fold(txt, node.child_by_field_name(f), sub, picks) for f in ("consequence", "alternative"))
    return (a if a == b else None) if c is None else b if c == 0 else a
  if node.type == "number_literal":
    m = re.fullmatch(r"(0[xX][0-9a-fA-F]+|0|[1-9][0-9]*)[uU]?", text(node))
    v = int(m[1], 0) if m else None
  elif node.type == "binary_expression":
    op, a, b = text(node.child_by_field_name("operator")), *(_fold(txt, node.child_by_field_name(f), sub, picks) for f in ("left", "right"))
    if POSITIVE in (a, b):  # of an unknown value, only 0 * x, x * 0, 0 / x and 0 % x are known
      return 0 if (op == "*" and 0 in (a, b)) or (op in ("/", "%") and a == 0) else None
    if a is None or b is None or op not in FOLD_OPS or (op in ("/", "%") and b == 0) or (op in ("<<", ">>") and b >= 31):
      return None
    v = int(FOLD_OPS[op](a, b))
  else:
    return None
  return v if v is not None and 0 <= v < 2 ** 31 else None


def _ways(txt, node, sub, op_start):
  """`node`'s values for each way its conditionals with a condition that isn't constant (and doesn't hold the mutation)
  can go, None if any doesn't fold."""
  conds, stack = [], [node]
  while stack:
    n = stack.pop()
    stack.extend(n.children)
    cond = n.child_by_field_name("condition") if n.type == "conditional_expression" else None
    if cond is not None and not cond.start_byte <= op_start < cond.end_byte and _fold(txt, cond, {}, {}) is None:
      conds.append(n.start_byte)
  values = [_fold(txt, node, sub, dict(zip(conds, way, strict=True))) for way in itertools.product((False, True), repeat=len(conds))]
  return None if None in values else values


def _equivalent(txt, node, op_start, sub):
  """Whether the mutation (`sub`, as _fold takes it) doesn't change the value of the outermost integer constant
  expression around it, for any way its conditionals go, e.g. 3U / 8U, (x ? 28U : 47U) / 8U or (x == 1 ? 5U : 5U), or
  turns x + 0 into x - 0 (or back), with 0 any constant expression folding to 0."""
  if node.type == "binary_expression" and node.child_by_field_name("operator").start_byte == op_start and \
     {txt[op_start], sub.get(op_start)} == {"+", "-"} and _fold(txt, node.child_by_field_name("right"), {}, {}) == 0:
    return True  # for floating point x this ignores the sign of a zero result, which nothing here depends on
  top = None
  while node is not None and node.type.endswith(("_expression", "_literal")):
    if _ways(txt, node, {}, op_start) is not None:
      top = node
    node = node.parent
  return top is not None and _ways(txt, top, {}, op_start) == _ways(txt, top, sub, op_start)


def _bits(txt, node):
  """A mask of the bits `node`'s integer value can have, -1 for any: from 8-bit CAN data, constants, GET_BYTES_* of a
  constant length, and the masks and shifts applied to them."""
  def field(f):
    return node.child_by_field_name(f)
  if node.type == "parenthesized_expression" and node.named_child_count == 1:
    return _bits(txt, node.named_children[0])
  if node.type == "subscript_expression" and field("argument").type == "field_expression" and \
     field("argument").child_by_field_name("field").text == b"data":
    return 0xFF  # CANPacket_t.data
  if node.type == "call_expression" and field("function").text in (b"GET_BYTES_LE", b"GET_BYTES_BE"):
    n = _fold(txt, field("arguments").named_children[2], {}, {})
    return (1 << (8 * n)) - 1 if isinstance(n, int) and n <= 4 else -1
  if node.type == "binary_expression":
    op, left, k = field("operator").type, _bits(txt, field("left")), _fold(txt, field("right"), {}, {})
    if op == "&":
      return left & _bits(txt, field("right"))
    if op in (">>", "<<") and left >= 0 and isinstance(k, int):
      return left >> k if op == ">>" else left << k
  v = _fold(txt, node, {}, {})
  return v if isinstance(v, int) else -1


def _demanded(txt, node):
  """A mask of the bits of `node`'s integer value that can matter, -1 for all: those a mask, a shift, or a cast to or a
  declaration of an 8 or 16-bit unsigned integer keeps (through the operators carrying bits only upward)."""
  parent = node.parent
  while parent is not None and parent.type == "parenthesized_expression":
    node, parent = parent, parent.parent
  narrow = {b"uint8_t": 0xFF, b"uint16_t": 0xFFFF}
  if parent is not None and parent.type == "cast_expression":
    return narrow.get(parent.child_by_field_name("type").text, -1) & _demanded(txt, parent)
  if parent is not None and parent.type == "init_declarator" and parent.parent.type == "declaration":
    return narrow.get(parent.parent.child_by_field_name("type").text, -1)
  if parent is None or parent.type != "binary_expression":
    return -1
  op, left, right = parent.child_by_field_name("operator").type, parent.child_by_field_name("left"), parent.child_by_field_name("right")
  k, outer = _fold(txt, right if left == node else left, {}, {}), _demanded(txt, parent)
  if op == "&" and isinstance(k, int):
    return k & outer
  if op in (">>", "<<") and left == node and isinstance(k, int):
    return outer << k if op == ">>" else outer >> k
  if op in ("+", "-", "*", "|", "^") and outer >= 0:
    return (1 << outer.bit_length()) - 1  # a carry only moves bits up
  return -1


def _parseable(txt):
  """The preprocessed source as tree-sitter can parse it, with the same offsets: line markers blanked, and every
  __typeof__(...) (whose operand is never evaluated) replaced by int."""
  txt = re.sub(r"^#.*", lambda m: " " * len(m[0]), txt, flags=re.MULTILINE)
  while m := re.search(r"__typeof__\(", txt):
    depth, end = 0, m.end() - 1
    while depth := depth + {"(": 1, ")": -1}.get(txt[end], 0):
      end += 1
    txt = txt[:m.start()] + "int".ljust(end + 1 - m.start()) + txt[end + 1:]
  return txt


def _identity(txt, node, op, deleted, kept):
  """Whether deleting operand `deleted` of `node` leaves its value unchanged: the 0 in x | 0 (or ^, << and >>), or a
  mask keeping every bit the other operand can have that matters, e.g. the 0xFU in (msg->data[0] >> 4) & 0xFU."""
  value = _fold(txt, deleted, {}, {})
  if op == "&":
    return isinstance(value, int) and _bits(txt, kept) & _demanded(txt, node) & ~value == 0
  return value == 0 and op in ("|", "^", "<<", ">>")


def find_sites(txt):
  # (file, line) of each preprocessed line from the line markers, with file None if it's not safety code
  origins, origin, files = [], (None, 0), {}
  for line in txt.splitlines():
    if m := re.match(r'# (\d+) "([^"]+)"', line):
      if m[2] not in files:
        f = Path(m[2]).resolve()
        files[m[2]] = f if f.is_relative_to(SAFETY_DIR) and not f.is_relative_to(TESTS_DIR) else None
      origin = (files[m[2]], int(m[1]) - 1)
    else:
      origin = (origin[0], origin[1] + 1)
    origins.append(origin)
  sources, sites = {}, {}

  def add(node, op, mutator, mutated, static, sub=None):
    """Mutates `node` by replacing txt[op.start_byte:op.end_byte] (op is a node or a (start, end, row) span)."""
    start, end, row = (op.start_byte, op.end_byte, op.start_point[0]) if isinstance(op, ts.Node) else op
    file, line = origins[row]
    if file is None or (mutator not in STATEMENT_MUTATORS and _equivalent(txt, node, start, sub or {start: mutated})):
      return
    assert static != "unsupported", f"can only mutate a static declaration of one named object: {file}:{line}"
    # compilers disagree on which line a backslash-continued line maps to; use its first line
    lines = sources.setdefault(file, file.read_text().splitlines())
    while line > 1 and lines[line - 2].rstrip().endswith("\\"):
      line -= 1
    scope = node
    while scope.parent is not None and scope.parent.type != "translation_unit":
      scope = scope.parent
    name = scope.child_by_field_name("declarator")
    while name is not None and name.type != "identifier":
      name = name.child_by_field_name("declarator")
    sites[(start, mutator, end)] = (node.start_byte, node.end_byte, start, end, txt[start:end], mutated, mutator, file, line, static,
                                    "" if name is None else name.text.decode())

  def delete(node, mutator):  # skip a statement
    add(node, node, mutator, "", None)

  def deletable(node):
    """The statement mutator that deletes `node`, if any; a body that only holds such a statement isn't deleted again."""
    if node.parent is None or node.parent.type not in STATEMENT_PARENTS:
      return None
    if node.type == "expression_statement" and node.named_child_count > 0:
      expr = node.named_children[0]
      while expr.type == "parenthesized_expression":
        expr = expr.named_children[0]
      void = expr.type == "cast_expression" and expr.child_by_field_name("type").text == b"void"
      return None if void else "delete_statement"  # (void)x; does nothing
    if node.type == "return_statement" and node.named_child_count == 0:  # in a void function, if not its last statement
      body = node.parent
      return None if body.parent.type == "function_definition" and body.named_children[-1] == node else "delete_return"
    if node.type == "compound_statement" or (node.type == "if_statement" and node.parent.type == "else_clause"):
      if node.parent.type not in ("if_statement", "else_clause"):
        return None
      inner = [c for c in node.named_children if c.type != "comment"] if node.type == "compound_statement" else [node]
      if not inner or (len(inner) == 1 and inner[0] is not node and deletable(inner[0]) is not None):
        return None
      # Not a body returning a value where skipping it falls off the end of the function, nor one declaring a static
      # object, whose mutated copies (see instrument) are inserted in it.
      func = node.parent
      while func.type != "function_definition":
        func = func.parent
      returns, declares_static, stack = False, False, [node]
      while stack:
        n = stack.pop()
        returns |= n.type == "return_statement" and n.named_child_count > 0
        declares_static |= n.type == "storage_class_specifier" and n.text == b"static"
        stack.extend(n.children)
      last = func.child_by_field_name("body").named_children[-1]
      return None if declares_static or (returns and last.type != "return_statement") else "delete_branch"
    return None

  stack = [(ts.Parser(ts.Language(ts_c.language())).parse(_parseable(txt).encode()).root_node, None)]
  while stack:
    node, static = stack.pop()
    if node.type in UNEVALUATED:
      continue
    parent = node.parent
    if node.type == "declaration" and parent is not None and (parent.type == "translation_unit" or any(
        c.type == "storage_class_specifier" and c.text == b"static" for c in node.children)):
      # A static object's initializer must be constant, so its sites are mutated in copies of the object (see instrument)
      declarators = node.children_by_field_name("declarator")
      name = declarators[0] if len(declarators) == 1 else None
      while name is not None and name.type != "identifier":
        name = name.child_by_field_name("declarator")
      scope_end = len(txt) if parent.type == "translation_unit" else parent.end_byte - 1
      static = "unsupported" if name is None else (node.start_byte, node.end_byte, name.start_byte, name.end_byte, scope_end)
    stack.extend((c, static) for c in node.children)
    if node.type.endswith("statement") and (mutator := deletable(node)):
      delete(node, mutator)
    op = node.child_by_field_name("operator")
    if op is None:
      continue
    if (node.type, op.type) in RULES:
      add(node, op, *RULES[(node.type, op.type)], static)
    if node.type == "assignment_expression" and op.type in COMPOUND_ASSIGNMENTS:
      add(node, op, "delete_operator", "=", static)
    if node.type == "binary_expression" and op.type in DELETE_OPERAND:
      left, _, right = node.children  # the preprocessed source has no comments
      for side, kept, deleted, span in (("R", left, right, (left.end_byte, node.end_byte, op.start_point[0])),
                                        ("L", right, left, (node.start_byte, right.start_byte, node.start_point[0]))):
        # deleting the operation keeps its variable operand: x & 0xFU -> x, not 0xFU
        if side in DELETE_OPERAND[op.type] and _fold(txt, kept, {}, {}) is None and not _identity(txt, node, op.type, deleted, kept):
          add(node, span, "delete_operand", "", static, {(node.start_byte, node.end_byte): "left" if kept == left else "right"})
    if node.type == "binary_expression" and op.type in COMPARISONS:  # boundary: integer literals in the operands
      operands = [n for n in (node.child_by_field_name("left"), node.child_by_field_name("right")) if n is not None]
      while operands:
        n = operands.pop()
        if n.type == "number_literal" and (mutated := _boundary(txt[n.start_byte:n.end_byte])):
          add(n, n, "boundary", mutated, static)
        elif n.type not in UNEVALUATED:
          operands.extend(n.children)
  return [Site(i, *sites[k]) for i, k in enumerate(sorted(sites))]


# ---------------------------------------------------------------- mutated library

PRELUDE = """
static int __mutation_id = -1;
static unsigned char __mutation_hit[{n}];
void mutation_set_active_mutant(int id) {{ __mutation_id = id; }}
unsigned char *mutation_hits(void) {{ return __mutation_hit; }}
void __sanitizer_set_death_callback(void (*callback)(void));
void _exit(int status);
static void __mutation_ubsan_died(void) {{ _exit({code}); }}
__attribute__((constructor)) static void __mutation_init(void) {{ __sanitizer_set_death_callback(__mutation_ubsan_died); }}
"""


def instrument(txt, sites):
  """The source with each site's expression E replaced by `mutant active ? mutated E : E`, and its evaluation recorded.
  A statement site's statement S becomes `if (mutant not active) S` instead.

  The mutated form only runs while its site is the active mutant, so the sites nested in it aren't instrumented there.
  The result is converted back to E's type, so the surrounding expressions are unchanged."""
  def rewrite(lo, hi, inner):  # txt[lo:hi] with the (sorted, non-overlapping or nested) sites in it instrumented
    parts, i = [], 0
    while i < len(inner):
      s, j = inner[i], i + 1
      while j < len(inner) and inner[j].start < s.end:
        j += 1
      e, original = txt[s.start:s.end], rewrite(s.start, s.end, inner[i + 1:j])
      if s.mutator in STATEMENT_MUTATORS:
        parts += [txt[lo:s.start], f"{{ __mutation_hit[{s.id}] = 1; if (__mutation_id != {s.id}) {{ {original} }} }}"]
      else:
        mutated = txt[s.start:s.op_start] + s.mutated + txt[s.op_end:s.end]
        selected = f"(__mutation_hit[{s.id}] = 1, __mutation_id == {s.id}) ? ({mutated}) : ({original})"
        parts += [txt[lo:s.start], f"((__typeof__(1 ? ({e}) : ({e})))({selected}))"]
      lo, i = s.end, j
    return "".join(parts) + txt[lo:hi]

  # A static object's initializer must be constant, so each of its sites k gets a copy of the declaration with just
  # that mutation, __mutation_static_k, and the object's uses after it are redirected to the copy while k is active.
  inserts, static_sites = [(len(txt), "")], {}
  for s in sites:
    if s.static is not None:
      static_sites.setdefault(s.static, []).append(s)
  for (start, end, name_start, name_end, scope_end), group in static_sites.items():
    name = txt[name_start:name_end]
    copies = "".join(txt[start:name_start] + f"__mutation_static_{s.id}" + txt[name_end:s.op_start] + s.mutated + txt[s.op_end:end] + "\n"
                     for s in group)
    redirect = "".join(f"__mutation_hit[{s.id}] = 1, " for s in group)
    redirect += "".join(f"__mutation_id == {s.id} ? &__mutation_static_{s.id} : " for s in group)
    inserts += [(end, f"\n{copies}#define {name} (*({redirect}&{name}))\n"), (scope_end, f"\n#undef {name}\n")]
  out, lo = [PRELUDE.format(n=max(len(sites), 1), code=UBSAN_EXIT_CODE)], 0
  runtime = sorted((s for s in sites if s.static is None), key=lambda s: (s.start, -s.end))
  for at, text in sorted(inserts):  # never inside a runtime site: they're at the end of a declaration or scope
    out += [rewrite(lo, at, [s for s in runtime if lo <= s.start < at]), text]
    lo = at
  return "".join(out)


# ---------------------------------------------------------------- tests, run in forked children

TESTS = {}  # test id -> TestCase, in the parent and inherited by every child
PRISTINE = {}  # test class -> its scalar class attributes, which some tests use as counters (`self.__class__.cnt += 1`)
SCALARS = (int, float, bool, str, type(None))


def discover_tests():
  def flatten(suite):
    for t in suite:
      yield from flatten(t) if isinstance(t, unittest.TestSuite) else [t]
  for f in sorted(TESTS_DIR.glob("test_*.py")):
    if f.name not in EXCLUDED_TESTS:
      for t in flatten(unittest.defaultTestLoader.loadTestsFromName(".".join(f.relative_to(ROOT).with_suffix("").parts))):
        TESTS[t.id()] = t
  # setUpClass only skips helper classes (and has no other effect), so it's run once here instead of in every child
  for cls in {type(t) for t in TESTS.values()}:
    try:
      cls.setUpClass()
    except unittest.SkipTest:
      for tid in [tid for tid, t in TESTS.items() if type(t) is cls]:
        del TESTS[tid]
  from opendbc.can.dbc import DBC
  for cls in {type(t) for t in TESTS.values()}:
    PRISTINE.update({c: {k: v for k, v in vars(c).items() if type(v) in SCALARS} for c in cls.__mro__})
    if isinstance(getattr(cls, "DBC", None), str):
      DBC(cls.DBC)  # a read-only cache, built here so not every child builds it


def run_test(tid, result):
  """Runs a test as if it were the first one run: class attributes as discovered and `random` seeded by its id."""
  test = TESTS[tid]
  for cls, state in ((c, PRISTINE[c]) for c in type(test).__mro__):
    for k in [k for k, v in vars(cls).items() if type(v) in SCALARS and k not in state]:
      delattr(cls, k)
    for k, v in state.items():
      if vars(cls).get(k, not v) is not v:
        setattr(cls, k, v)
  random.seed(tid)
  test.run(result)


def baseline_task(tids, n):
  """Runs tests with no mutant active, returning {test id: (CPU seconds, ids of the sites it reaches)}."""
  import numpy as np
  signal.setitimer(signal.ITIMER_PROF, 0)  # no CPU limit: the original code doesn't hang
  hits = np.frombuffer(libsafety_py.ffi.buffer(libsafety_py.libsafety.mutation_hits(), n), dtype=np.uint8)
  out, result = {}, unittest.TestResult()
  for tid in tids:
    hits[:], t0 = 0, time.process_time()
    run_test(tid, result)
    if not result.wasSuccessful():
      return f"{tid} failed:\n{(result.failures + result.errors + [(None, 'unexpected success')])[0][1]}"
    if not result.skipped or result.skipped[-1][0] is not TESTS[tid]:
      out[tid] = (time.process_time() - t0, np.flatnonzero(hits).tolist())
  return out


def mutant_task(site_id, tids, cpu_limits):
  """Runs tests with the mutant active until one fails; returns (that test's id or None, CPU seconds spent in tests)."""
  libsafety_py.libsafety.mutation_set_active_mutant(site_id)
  result, t0 = unittest.TestResult(), time.process_time()
  for tid in tids:
    # SIGPROF's default action ends the process, even in a C loop
    signal.setitimer(signal.ITIMER_PROF, cpu_limits[tid])
    run_test(tid, result)
    signal.setitimer(signal.ITIMER_PROF, CPU_LIMIT_FIXTURE)
    if not result.wasSuccessful():
      return tid, time.process_time() - t0
  return None, time.process_time() - t0


def run_forked(task, args, wall_limit):
  """Runs task(*args) in a fork of this pool worker. Returns ("ok", its return value), ("killed", how the child died
  of something its own code did) or (INFRA, why there's no result)."""
  r, w = os.pipe()
  pid = os.fork()
  if pid == 0:
    try:
      os.close(r)
      resource.setrlimit(resource.RLIMIT_CORE, (0, 0))  # crash reporters can take seconds per crash
      if sys.platform == "linux":  # macOS doesn't enforce RLIMIT_AS
        libc = ctypes.CDLL(None)
        libc.prctl(4, 0, 0, 0, 0)  # PR_SET_DUMPABLE: no core dumps, not even piped ones
        libc.prctl(1, signal.SIGKILL, 0, 0, 0)  # PR_SET_PDEATHSIG: don't outlive the worker
        vm = int(Path("/proc/self/statm").read_text().split()[0]) * os.sysconf("SC_PAGE_SIZE")
        resource.setrlimit(resource.RLIMIT_AS, (vm + MEM_HEADROOM, vm + MEM_HEADROOM))
      devnull = os.open(os.devnull, os.O_WRONLY)
      os.dup2(devnull, 1)
      os.dup2(devnull, 2)
      signal.setitimer(signal.ITIMER_PROF, CPU_LIMIT_FIXTURE)
      msg = ("ok", task(*args))
    except BaseException:
      msg = (INFRA, traceback.format_exc())
    signal.setitimer(signal.ITIMER_PROF, 0)
    with os.fdopen(w, "wb") as f:
      f.write(pickle.dumps(msg))
    os._exit(0)
  os.close(w)
  data, deadline = b"", time.monotonic() + wall_limit
  while not (timed_out := not select.select([r], [], [], max(0.0, deadline - time.monotonic()))[0]):
    if not (chunk := os.read(r, 1 << 20)):
      break  # the child only closes its end by exiting
    data += chunk
  os.close(r)
  if timed_out:
    os.kill(pid, signal.SIGKILL)
  status = os.waitpid(pid, 0)[1]
  if timed_out:
    return INFRA, f"no result after {wall_limit:.0f}s"
  if os.WIFSIGNALED(status):
    sig = signal.Signals(os.WTERMSIG(status))
    if sig == signal.SIGPROF:
      return "killed", "timeout"
    return ("killed", f"crash: {sig.name}") if sig in CRASH_SIGNALS else (INFRA, f"killed by {sig.name} from outside")
  if os.WEXITSTATUS(status) == UBSAN_EXIT_CODE:
    return "killed", "ubsan"
  try:
    return pickle.loads(data)
  except Exception:
    return INFRA, f"exited with code {os.WEXITSTATUS(status)} without a result"


# ---------------------------------------------------------------- reporting

def site_keys(sites):
  """{site id: how KNOWN_SURVIVORS lists it}, which doesn't change with the lines around it: `file function [mutator]
  change`, and `#n` after it for the n-th site that's the same."""
  keys, seen = {}, Counter()
  for s in sites:
    key = f"{s.file.relative_to(SAFETY_DIR)} {s.function} [{s.mutator}] {s.change()}"
    seen[key] += 1
    keys[s.id] = key if seen[key] == 1 else f"{key} #{seen[key]}"
  return keys


def known_survivors(sites):
  """The ids of the sites listed in KNOWN_SURVIVORS, and the listed lines that match no site. Whitespace is ignored,
  since compilers preprocess macros into differently spaced text."""
  def norm(key):
    return re.sub(r"\s", "", key)
  listed = [ln for ln in KNOWN_SURVIVORS.read_text().splitlines() if ln.strip() and not ln.startswith("#")]
  ids = {norm(k): i for i, k in site_keys(sites).items()}
  return {ids[norm(ln)] for ln in listed if norm(ln) in ids}, [ln for ln in listed if norm(ln) not in ids]


def snippet(site, sites, context=2):
  """The site's source line and a few around it, with the mutation marked in place."""
  lines = site.file.read_text().splitlines()
  i = site.line - 1
  # the n-th standalone occurrence of the token on the line, for the n-th site replacing that token there
  n = sum(1 for s in sites if (s.file, s.line, s.original) == (site.file, site.line, site.original) and s.id < site.id)
  if site.mutator == "boundary":
    pattern = r"(?<![\w.])" + re.escape(site.original) + r"(?![\w.])"
  elif site.mutator.startswith("delete") and not site.mutated:
    pattern = re.escape(site.change()[1:])
  else:
    pattern = r"(?<![-+*/%&|^<>=!])" + re.escape(site.original) + r"(?![-+*/%&|^<>=])"
  found, marker = list(re.finditer(pattern, lines[i])), colorize(f"[[{site.change(60)}]]", RED)
  if n < len(found):
    lines[i] = lines[i][:found[n].start()] + marker + lines[i][found[n].end():]
  else:
    lines[i] += f"  <- {marker} in a macro expansion"
  lo, hi = max(0, i - context), min(len(lines), i + context + 1)
  width = len(str(hi))
  return "\n".join(f"{'>' if k == i else ' '} {k + 1:>{width}} | {lines[k]}" for k in range(lo, hi))


def progress(results, total, elapsed):
  counts = Counter(outcome for outcome, _, _ in results.values())
  done, rate = len(results), len(results) / max(elapsed, 1e-9)
  bar = "#" * (30 * done // total) + "-" * (30 - 30 * done // total)
  k, s, i = colorize(f"k:{counts[KILLED]}", GREEN), colorize(f"s:{counts[SURVIVED] + counts[UNCOVERED]}", RED), colorize(f"i:{counts[INFRA]}", YELLOW)
  return f"[{bar}] {done}/{total} {k} {s} {i} mps:{rate:.1f} elapsed:{elapsed:.1f}s eta:{(total - done) / max(rate, 1e-9):.1f}s"


# ---------------------------------------------------------------- main

def main():
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("-j", type=int, default=os.cpu_count() or 1, help="parallel jobs")
  parser.add_argument("--only", type=int, nargs="+", help="only run these mutant ids")
  parser.add_argument("--list", action="store_true", help="list the mutants and exit")
  parser.add_argument("--known", action="store_true", help=f"also run the known survivors in {KNOWN_SURVIVORS.name}")
  args = parser.parse_args()
  start = time.monotonic()

  with tempfile.TemporaryDirectory() as tmp:
    txt = subprocess.run(["cc", "-E", "-std=gnu11", "-nostdlib", "-fno-builtin", "-DALLOW_DEBUG", f"-I{ROOT}", f"-I{SAFETY_DIR / 'board'}",
                          str(TESTS_DIR / "libsafety/safety.c")], check=True, capture_output=True, text=True).stdout
    all_sites = find_sites(txt)
    sites = [s for s in all_sites if args.only is None or s.id in args.only]
    print(f"Found {len(all_sites)} mutants: " + ", ".join(f"{k} ({v})" for k, v in Counter(s.mutator for s in all_sites).items()), flush=True)
    if args.list:
      print("\n".join(map(str, sites)))
      return 0
    if not sites or len(sites) != len(set(args.only or [s.id for s in sites])):
      print("--only: no such mutant ids")
      return 2
    known, stale = known_survivors(all_sites)
    keys = site_keys(all_sites)
    if stale:
      print(colorize(f"No such mutants, remove them from {KNOWN_SURVIVORS.relative_to(ROOT)}:", RED) + "".join(f"\n- {k}" for k in stale))
      return 1
    if args.only is None and not args.known:
      sites = [s for s in sites if s.id not in known]
      print(f"Skipping {len(known)} known survivors listed in {KNOWN_SURVIVORS.relative_to(ROOT)} (--known runs them)", flush=True)

    # every site is instrumented, so the ids and the baseline don't depend on --only
    lib = Path(tmp) / "libsafety_mutation.so"
    (Path(tmp) / "mutation.c").write_text(instrument(txt, all_sites))
    cc = subprocess.Popen(["cc", *CFLAGS, str(Path(tmp) / "mutation.c"), "-o", str(lib)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    discover_tests()
    cc_output = cc.communicate()[0].decode()
    if cc.returncode != 0:
      print(cc_output[-4000:] + "\nFailed to build the mutation library")
      return 2
    libsafety_py.ffi.cdef("unsigned char *mutation_hits(void);")
    libsafety_py.load(lib)  # only called into in forked children

    gc.collect()
    gc.freeze()  # children touching these objects' GC headers would copy their memory pages
    # a worker never runs mutated code, so it only dies if killed from outside, which fails the run (BrokenProcessPool)
    with ProcessPoolExecutor(args.j, mp_context=multiprocessing.get_context("fork")) as pool:
      def run_all(tasks):  # {key: (task, args, wall limit)}, started in order -> (key, status, value) as they finish
        futures = {pool.submit(run_forked, *task): key for key, task in tasks.items()}
        for f in as_completed(futures):
          yield futures[f], *f.result()

      # Baseline: every test must pass with no mutant active. Small chunks balance well across the workers.
      tids, coverage = list(TESTS), {}
      for _, status, value in run_all({i: (baseline_task, (tids[i:i + 16], len(all_sites)), 600) for i in range(0, len(tids), 16)}):
        if status != "ok" or isinstance(value, str):
          print(f"Baseline failed with no mutant active ({status}):\n{value}")
          return 2
        coverage.update(value)
      print(f"Baseline: {len(coverage)} tests passed in {time.monotonic() - start:.1f}s", flush=True)
      print(f"Running {len(sites)} mutants with {args.j} workers", flush=True)

      reaching = {}
      for tid, (_, hits) in coverage.items():
        for k in hits:
          reaching.setdefault(k, []).append(tid)
      results, tasks = {}, []
      for s in sites:
        if s.id not in reaching:
          results[s.id] = (UNCOVERED, "no test reaches it", 0.0)
          continue
        # own test module first, then the quickest first; the order only affects speed and which test kills it
        own = f"opendbc.safety.tests.test_{s.file.stem}."
        tids = sorted(reaching[s.id], key=lambda t: (not t.startswith(own), coverage[t][0], t))
        # a test taking far longer than on the original code counts as a hang
        limits = {t: 1.0 + 20 * coverage[t][0] for t in tids}
        cpu = sum(coverage[t][0] for t in tids)
        tasks.append((cpu, s.id, (mutant_task, (s.id, tids, limits), 10 + 4 * (cpu + max(limits.values()) + CPU_LIMIT_FIXTURE))))
      tasks.sort(key=lambda t: -t[0])  # the most expensive first, so they don't make up the tail
      mutants_start = time.monotonic()
      for site_id, status, value in run_all({k: task for _, k, task in tasks}):
        if status == "ok":
          results[site_id] = ((KILLED, f"test: {value[0]}") if value[0] else (SURVIVED, "no test fails")) + (value[1],)
        else:
          results[site_id] = (KILLED if status == "killed" else INFRA, value, 0.0)
        if sys.stdout.isatty():
          print("\r" + progress(results, len(sites), time.monotonic() - mutants_start), end="", flush=True)
      print(("\r" if sys.stdout.isatty() else "") + progress(results, len(sites), time.monotonic() - mutants_start), flush=True)

  for s in sites:  # a known survivor surviving isn't a failure, but being killed is: it should leave the list
    if s.id in known and results[s.id][0] in (KILLED, SURVIVED):
      results[s.id] = (KNOWN if results[s.id][0] == SURVIVED else UNKNOWN, *results[s.id][1:])
  counts = Counter(outcome for outcome, _, _ in results.values())
  for title, outcome, color in (("Surviving mutants", SURVIVED, RED), ("Uncovered mutants (no test reaches them)", UNCOVERED, RED),
                                (f"Killed known survivors (remove them from {KNOWN_SURVIVORS.name})", UNKNOWN, RED),
                                ("Infra errors", INFRA, YELLOW)):
    bad = [s for s in sites if results[s.id][0] == outcome]
    if bad:
      print("\n" + colorize(title, color))
    for s in bad:
      print(f"- {s}")
      print("    " + results[s.id][1].strip().replace("\n", "\n    ") if outcome == INFRA else snippet(s, all_sites))
      if outcome == SURVIVED:
        print(f"    {KNOWN_SURVIVORS.name}: {keys[s.id]}")
  kills = Counter(detail.split(":")[0] for outcome, detail, _ in results.values() if outcome == KILLED)
  test_sec, elapsed = sum(r[2] for r in results.values()), time.monotonic() - start
  print("\n" + colorize("Mutation summary", BOLD))
  print(f"  total: {len(sites)}")
  print(f"  killed: {colorize(counts[KILLED], GREEN)} (" + ", ".join(f"{k}: {v}" for k, v in kills.most_common()) + ")")
  print(f"  survived: {colorize(counts[SURVIVED], RED)}")
  print(f"  uncovered: {colorize(counts[UNCOVERED], RED)}")
  print(f"  known survivors: {len(known)} ({counts[KNOWN]} run and survived, {colorize(counts[UNKNOWN], RED)} run and killed)")
  print(f"  infra_error: {colorize(counts[INFRA], YELLOW)}")
  print(f"  test_time_sum: {test_sec:.2f}s")
  print(f"  avg_test_per_mutant: {test_sec / len(sites):.3f}s")
  print(f"  mutants_per_second: {len(sites) / elapsed:.2f}")
  print(f"  elapsed: {elapsed:.2f}s")
  return 2 if counts[INFRA] else 1 if counts[SURVIVED] + counts[UNCOVERED] + counts[UNKNOWN] else 0


if __name__ == "__main__":
  raise SystemExit(main())
