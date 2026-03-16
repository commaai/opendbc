class VLDict(dict):
  def __init__(self, parser):
    super().__init__()
    self.parser = parser

  def __getitem__(self, key):
    if key not in self:
      self.parser._add_message(key)
    return super().__getitem__(key)
