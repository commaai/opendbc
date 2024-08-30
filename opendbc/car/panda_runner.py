import time
from contextlib import AbstractContextManager

from opendbc_repo.opendbc.car.structs import CarState, CarControl
from panda import Panda
from opendbc.car.car_helpers import get_car
from opendbc.car.can_definitions import CanData

class PandaRunner(AbstractContextManager):
  def __enter__(self):
    self.p = Panda()
    self.p.reset()

    # setup + fingerprinting
    p.set_safety_mode(Panda.SAFETY_ELM327, 1)
    self.CI = get_car(self._can_recv, p.can_send_many, p.set_obd, True)
    print("fingerprinted", self.CI.CP.carName)
    assert self.CI.CP.carFingerprint != "mock", "Unable to identify car. Check connections and ensure car is supported."

    p.set_safety_mode(Panda.SAFETY_ELM327, 1)
    self.CI.init(self.CI.CP, self._can_recv, p.can_send_many)
    p.set_safety_mode(Panda.SAFETY_TOYOTA, self.CI.CP.safetyConfigs[0].safetyParam)

    return self

  def __exit__(self, exc_type, exc_value, traceback):
    self.p.set_safety_mode(Panda.SAFETY_NOOUTPUT)
    self.p.reset()  # avoid siren
    return super().__exit__(exc_type, exc_value, traceback)

  @property
  def panda(self) -> Panda:
    return self.p

  def _can_recv(self, wait_for_one: bool = False) -> list[list[CanData]]:
    recv = self.p.can_recv()
    while len(recv) == 0 and wait_for_one:
      recv = self.p.can_recv()
    return [[CanData(addr, dat, bus) for addr, dat, bus in recv], ]

  def read(self) -> CarState:
    cd = [CanData(addr, dat, bus) for addr, dat, bus in p.can_recv()]
    cs = self.CI.update([int(time.monotonic()*1e9), cd])
    assert cs.canValid, "CAN went invalid, check connections"
    return cs

  def write(self, cc: CarControl) -> None:
    _, can_sends = self.CI.apply(cc)
    p.can_send_many(can_sends, timeout=20)
    p.send_heartbeat()

if __name__ == "__main__":
  with PandaRunner() as p:
    print(p.read())