from contextlib import contextmanager

from panda import Panda
from opendbc.car.car_helpers import get_car
from opendbc.car.can_definitions import CanData

@contextmanager
def PandaRunner():
  p = Panda()

  def _can_recv(wait_for_one: bool = False) -> list[list[CanData]]:
    recv = p.can_recv()
    while len(recv) == 0 and wait_for_one:
      recv = p.can_recv()
    return [[CanData(addr, dat, bus) for addr, dat, bus in recv], ]

  try:
    # setup + fingerprinting
    p.set_safety_mode(Panda.SAFETY_ELM327, 1)
    CI = get_car(_can_recv, p.can_send_many, p.set_obd, True)
    print("fingerprinted", CI.CP.carName)

    p.set_safety_mode(Panda.SAFETY_ELM327, 1)
    CI.init(CI.CP, _can_recv, p.can_send_many)
    p.set_safety_mode(Panda.SAFETY_TOYOTA, CI.CP.safetyConfigs[0].safetyParam)

    yield p
  finally:
    p.set_safety_mode(Panda.SAFETY_NOOUTPUT)


if __name__ == "__main__":
  with PandaRunner() as p:
    print(p.can_recv())