import sys


def parameterized_class(attrs, values=None):
  """Class decorator that generates subclasses with different class attributes.

  Usage:
    @parameterized_class([{"x": 1}, {"x": 2}])
    @parameterized_class('x', [(1,), (2,)])
  """
  if isinstance(attrs, str):
    attrs = [attrs]
    params = [dict(zip(attrs, v, strict=True)) for v in values]
  else:
    params = attrs

  def decorator(cls):
    module = sys.modules[cls.__module__]
    for param_set in params:
      name = f"{cls.__name__}_{'_'.join(str(v) for v in param_set.values())}"
      new_cls = type(name, (cls,), param_set)
      new_cls.__qualname__ = name
      new_cls.__module__ = cls.__module__
      new_cls.__test__ = True
      setattr(module, name, new_cls)
    cls.__test__ = False
    return cls

  return decorator
