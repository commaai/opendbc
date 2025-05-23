#!/usr/bin/env python3
import os
import re
import tkinter as tk
from tkinter import ttk

BASEDIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
DBC_DIR = os.path.join(BASEDIR, 'opendbc', 'dbc')


def get_dbc_files():
  return sorted([f for f in os.listdir(DBC_DIR) if f.endswith('.dbc')])


def parse_messages(dbc_path):
  messages = []
  bo_pattern = re.compile(r'^BO_\s+(\d+)\s+([^:]+):')
  with open(dbc_path, encoding='utf-8') as f:
    for line in f:
      match = bo_pattern.match(line.strip())
      if match:
        msg_id = int(match.group(1))
        msg_name = match.group(2)
        messages.append((msg_id, msg_name))
  return messages


class AutocompleteCombobox(ttk.Combobox):
  def set_completion_list(self, completion_list):
    self._completion_list = sorted(completion_list, key=str.lower)
    self['values'] = self._completion_list
    self.bind('<KeyRelease>', self._check_key)

  def _check_key(self, event):
    typed = self.get()
    data = [item for item in self._completion_list if typed.lower() in item.lower()]
    self['values'] = data
    if event.keysym not in ('Return', 'Tab'):
      self.event_generate('<Down>')


def main():
  root = tk.Tk()
  root.title('DBC Viewer')

  dbc_files = get_dbc_files()

  combo = AutocompleteCombobox(root, width=60)
  combo.set_completion_list(dbc_files)
  combo.pack(padx=10, pady=10)

  messages_box = tk.Listbox(root, width=80)
  messages_box.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

  def load_messages(_=None):
    sel = combo.get()
    if not sel:
      return
    path = os.path.join(DBC_DIR, sel)
    messages = parse_messages(path)
    messages_box.delete(0, tk.END)
    for msg_id, msg_name in messages:
      messages_box.insert(tk.END, f"0x{msg_id:X} ({msg_id}) - {msg_name}")

  combo.bind('<<ComboboxSelected>>', load_messages)
  combo.bind('<Return>', load_messages)

  root.mainloop()


if __name__ == '__main__':
  main()
