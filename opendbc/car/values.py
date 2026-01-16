from typing import get_args

def __getattr__(name):
  if name in (n := {
    'BODY': 'body', 'CHRYSLER': 'chrysler', 'FORD': 'ford', 'GM': 'gm', 'HONDA': 'honda',
    'HYUNDAI': 'hyundai', 'MAZDA': 'mazda', 'MOCK': 'mock', 'NISSAN': 'nissan',
    'PSA': 'psa', 'RIVIAN': 'rivian', 'SUBARU': 'subaru', 'TESLA': 'tesla',
    'TOYOTA': 'toyota', 'VOLKSWAGEN': 'volkswagen'
  }):
    val = __import__(f'opendbc.car.{n[name]}.values', fromlist=['CAR']).CAR
  elif name == 'Platform':
    from opendbc.car.values import BODY, CHRYSLER, FORD, GM, HONDA, HYUNDAI, MAZDA, MOCK, NISSAN, PSA, RIVIAN, SUBARU, TESLA, TOYOTA, VOLKSWAGEN
    val = BODY | CHRYSLER | FORD | GM | HONDA | HYUNDAI | MAZDA | MOCK | NISSAN | PSA | RIVIAN | SUBARU | TESLA | TOYOTA | VOLKSWAGEN
  elif name == 'BRANDS':
    from opendbc.car.values import Platform
    val = get_args(Platform)
  elif name == 'PLATFORMS':
    from opendbc.car.values import BRANDS
    val = {str(platform): platform for brand in BRANDS for platform in brand}
  else:
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

  globals()[name] = val
  return val

def __dir__():
  return sorted(list(globals().keys()) + ['Platform', 'BRANDS', 'PLATFORMS',
    'BODY', 'CHRYSLER', 'FORD', 'GM', 'HONDA', 'HYUNDAI', 'MAZDA', 'MOCK', 'NISSAN',
    'PSA', 'RIVIAN', 'SUBARU', 'TESLA', 'TOYOTA', 'VOLKSWAGEN'])
