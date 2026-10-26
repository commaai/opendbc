"""Test-owned vehicle inputs for observable safety tests, including stock hardware.

The stream cycles start at counter zero on every regeneration. A transport must
retain its position by stream name when replacing payloads, so state changes do
not interrupt counters or drop unrelated vehicle signals.
"""
from dataclasses import dataclass, replace
from enum import IntEnum

from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.test_hyundai import TestHyundaiSafety, checksum
from opendbc.safety.tests.test_toyota import TestToyotaSafetyTorque


class Button(IntEnum):
  NONE = 0
  RESUME = 1
  SET = 2
  CANCEL = 4


@dataclass(frozen=True)
class VehicleState:
  cruise: bool = False
  button: Button = Button.NONE
  gas: bool = False
  brake: bool = False
  speed: float = 0.0  # m/s
  driver_torque: float = 0.0
  motor_torque: float = 0.0


class _RawPacker(CANPackerSafety):
  """Use fixture encoders without constructing packets for a particular DUT."""
  def make_can_msg_safety(self, name_or_addr, bus, values, fix_checksum=None):
    msg = self.make_can_msg(name_or_addr, bus, values)
    return fix_checksum(msg) if fix_checksum is not None else msg


def _frame(msg):
  address, data, bus = msg
  return [address, bytes(data).hex(), bus]


class VehicleProfile:
  fixture = None
  name = ""

  def __init__(self):
    self.state = VehicleState()
    # Several fixture encoders own counters on their class. Isolate those from
    # unittest collection, other profiles and command/fault packet generation.
    self._fixture_type = type(f"{self.name}InputEncoder", (self.fixture,), {})
    self.safety_model = int(self.fixture.SAFETY_MODEL)
    encoder = self._encoder()
    self.safety_param = int(encoder.SAFETY_PARAM)
    self.max_torque = self.fixture.MAX_TORQUE_LOOKUP[1][0]
    self.max_rate_up = self.fixture.MAX_RATE_UP
    self.max_rate_down = self.fixture.MAX_RATE_DOWN

  def _encoder(self):
    encoder = self._fixture_type(methodName="runTest")
    encoder.packer = _RawPacker(encoder.DBC)
    for key in ("cnt_gas", "cnt_speed", "cnt_brake", "cnt_cruise", "cnt_button"):
      setattr(self._fixture_type, key, 0)
    encoder._gas_pressed = self.state.gas
    encoder._cruise_on = self.state.cruise
    return encoder

  def update(self, **changes):
    if "button" in changes:
      changes["button"] = Button(changes["button"])
    self.state = replace(self.state, **changes)
    return self.streams()

  def streams(self):
    raise NotImplementedError

  # Test cadences stay within stock RX watchdog limits; they do not represent
  # production message frequencies. Measurement streams run at 50 Hz so the
  # six-sample torque windows fill within the 150 ms state-update settling time.
  def _stream(self, name, frequency, builder):
    return {"name": name, "period": 1.0 / frequency, "frames": [_frame(builder()) for _ in range(16)]}

  def torque(self, value, request=True):
    return _frame(self._encoder()._torque_cmd_msg(value, steer_req=int(request)))

  def corrupt(self, stream_name, index=0, byte=-1, mask=0xff):
    """Return an isolated damaged RX packet without changing background inputs."""
    stream = next(stream for stream in self.streams() if stream["name"] == stream_name)
    address, encoded, bus = stream["frames"][index % 16]
    data = bytearray.fromhex(encoded)
    data[byte] ^= mask
    return [address, data.hex(), bus]

  def engage_sequence(self):
    """State changes to apply in order, with at least one RX cycle per step."""
    return ({"cruise": False, "button": Button.SET}, {"cruise": True}, {"button": Button.NONE})


class ToyotaProfile(VehicleProfile):
  """Toyota LKA RX checks: AA speed, 260 torque, 1D2 PCM, 226 brake."""
  name = "toyota"
  fixture = TestToyotaSafetyTorque
  min_accel = fixture.MIN_ACCEL
  max_accel = fixture.MAX_ACCEL
  inactive_accel = fixture.INACTIVE_ACCEL

  def streams(self):
    e, s = self._encoder(), self.state
    return [self._stream("speed", 25, lambda: e._speed_msg(s.speed)),
            self._stream("torque", 50, lambda: e._torque_meas_msg(s.motor_torque, s.driver_torque)),
            self._stream("pcm", 25, lambda: e._pcm_status_msg(s.cruise)),
            self._stream("brake", 25, lambda: e._user_brake_msg(s.brake))]

  def accel(self, value):
    return _frame(self._encoder()._accel_msg(value))

  def button(self, value):
    if Button(value) != Button.CANCEL:
      raise ValueError("Toyota profile only supports a cancel command")
    return _frame(self._encoder()._accel_msg_343(0, cancel_req=1))

  def lta(self, req, req2, angle, winddown=100):
    return _frame(self._encoder()._lta_msg(req, req2, angle, torque_wind_down=winddown))

  def aeb(self, data):
    return [0x283, bytes(data).hex(), 0]

  def diagnostic(self, data=b"\x0f\x02\x3e\x00\x00\x00\x00\x00"):
    return [0x750, bytes(data).hex(), 0]


class HyundaiProfile(VehicleProfile):
  """Classic Hyundai RX checks: 260 gas, 386 speed, 394 brake,
  251 driver torque, 4F1 buttons and 421 PCM (all on bus zero).
  """
  name = "hyundai"
  fixture = TestHyundaiSafety

  def streams(self):
    e, s = self._encoder(), self.state

    def brake():
      # The unit fixture randomizes non-braking values; continuous profiles are
      # deterministic and still encode through the same packer and checksum.
      counter = e.__class__.cnt_brake
      e.__class__.cnt_brake += 1
      return e.packer.make_can_msg_safety("TCS13", 0, {"DriverOverride": 2 if s.brake else 0,
                                                        "AliveCounterTCS": counter % 8}, fix_checksum=checksum)

    return [self._stream("gas", 25, lambda: e._user_gas_msg(s.gas)),
            self._stream("speed", 25, lambda: e._speed_msg(s.speed * 3.6 / 0.03125)),
            self._stream("brake", 25, brake),
            self._stream("driver_torque", 50, lambda: e._torque_driver_msg(s.driver_torque)),
            self._stream("buttons", 25, lambda: e._button_msg(int(s.button))),
            self._stream("pcm", 25, lambda: e._pcm_status_msg(s.cruise))]

  def button(self, value):
    return _frame(self._encoder()._button_msg(int(Button(value))))


def get_vehicle_profile(name):
  profiles = {"toyota": ToyotaProfile, "hyundai": HyundaiProfile}
  try:
    return profiles[name]()
  except KeyError:
    raise ValueError(f"Unsupported vehicle profile {name!r}; choose from {tuple(profiles)}") from None
