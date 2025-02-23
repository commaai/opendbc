#!/usr/bin/env python3
import unittest

from opendbc.safety.safety.generator import generate_safety_config, PythonRenderer


class TestGenerator(unittest.TestCase):
  def setUp(self):
    self.maxDiff = None
    pass

  # Be a little more verbose to make it easy to update expected values.
  def assertMultiLineEqual(self, expected, actual, msg=None):
    msg = msg or f'actual:\n{actual}'
    super().assertMultiLineEqual(expected.strip(), actual.strip(), msg)


  def test_basic(self):
    CONDITIONS = [
      ["g0e0", "g0e1"],
      ["g1e0"],
    ]
    template = """
{% if g0e0 %}
  g0e0
{% elif g0e1 %}
  g0e1
{% else %}
  g0eN
{% endif %}
{% if g1e0 %}
  g1e0
{% endif %}
"""
    actual = generate_safety_config(template, CONDITIONS)

    expected = """
  if (g0e0) {
    if (g1e0) {
      // g0e0: True
      // g0e1: False
      // g1e0: True
        g0e0
        g1e0
    }
    else {
      // g0e0: True
      // g0e1: False
      // g1e0: False
        g0e0
    }
  }
  if (g0e1) {
    if (g1e0) {
      // g0e1: True
      // g0e0: False
      // g1e0: True
        g0e1
        g1e0
    }
    else {
      // g0e1: True
      // g0e0: False
      // g1e0: False
        g0e1
    }
  }
  else {
    if (g1e0) {
      // g0e0: False
      // g0e1: False
      // g1e0: True
        g0eN
        g1e0
    }
    else {
      // g0e0: False
      // g0e1: False
      // g1e0: False
        g0eN
    }
  }
"""
    self.assertMultiLineEqual(expected, actual)


  def test_basic_python(self):
    CONDITIONS = [
      ["g0e0", "g0e1"],
      ["g1e0"],
    ]
    template = """
if g0e0:
  add_rx_check("g0e0")
elif g0e1:
  add_rx_check("g0e1")
if g1e0:
  add_rx_check("g1e0")
"""
    actual = generate_safety_config(template, CONDITIONS, renderer=PythonRenderer)

    expected = """
  if (g0e0) {
    if (g1e0) {
      // g0e0: True
      // g0e1: False
      // g1e0: True
      g0e0
      g1e0
    }
    else {
      // g0e0: True
      // g0e1: False
      // g1e0: False
      g0e0
    }
  }
  if (g0e1) {
    if (g1e0) {
      // g0e1: True
      // g0e0: False
      // g1e0: True
      g0e1
      g1e0
    }
    else {
      // g0e1: True
      // g0e0: False
      // g1e0: False
      g0e1
    }
  }
  else {
    if (g1e0) {
      // g0e0: False
      // g0e1: False
      // g1e0: True
      g1e0
    }
    else {
      // g0e0: False
      // g0e1: False
      // g1e0: False
    }
  }
"""
    self.assertMultiLineEqual(expected, actual)


if __name__ == "__main__":
  unittest.main()
