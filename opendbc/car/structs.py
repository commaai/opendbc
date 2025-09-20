import os
from typing import Any

import capnpy
from capnpy import util as _capnpy_util
from opendbc.car.common.basedir import BASEDIR

# TODO: remove car from cereal/__init__.py and always import from opendbc
try:
  from cereal import car
except ImportError:
  # Pure-Python schema loading with capnpy (no C extensions)
  from capnpy.annotate import Options
  from capnpy.compiler.compiler import DEFAULT_OPTIONS

  # Disable convert_case to preserve pycapnp-style names (e.g., fwdRadar)
  OptEnum = type(DEFAULT_OPTIONS.convert_case)
  TTEnum = type(DEFAULT_OPTIONS.text_type)
  VCEnum = type(DEFAULT_OPTIONS.version_check)
  IRDEnum = type(DEFAULT_OPTIONS.include_reflection_data)
  options = Options(version_check=VCEnum.true, convert_case=OptEnum.false, text_type=TTEnum.unicode, include_reflection_data=IRDEnum.true)
  car = capnpy.load_schema(filename=os.path.join(BASEDIR, "car.capnp"), pyx=False, options=options)

# Keep original capnpy CarState types for reference
_CapnpCarState = car.CarState


class _CarStateShimFactory:
  # expose nested enum/struct placeholders compatible with existing code
  class ButtonEvent:
    class Type:
      unknown = 0
      leftBlinker = 1
      rightBlinker = 2
      accelCruise = 3
      decelCruise = 4
      cancel = 5
      lkas = 6
      altButton2 = 7
      mainCruise = 8
      setCruise = 9
      resumeCruise = 10
      gapAdjustCruise = 11

    def __init__(self, pressed: bool = False, ev_type: int = 0):
      self.pressed = pressed
      self.type = ev_type

  class WheelSpeeds:
    def __init__(self, fl=0.0, fr=0.0, rl=0.0, rr=0.0):
      self.fl, self.fr, self.rl, self.rr = fl, fr, rl, rr

  class CruiseState:
    def __init__(self):
      self.enabled = False
      self.speed = 0.0
      self.speedCluster = 0.0
      self.available = False
      self.standstill = False
      self.nonAdaptive = False

  class GearShifter:
    unknown = 0
    park = 1
    drive = 2
    neutral = 3
    reverse = 4
    sport = 5
    low = 6
    brake = 7
    eco = 8
    manumatic = 9

  def __call__(self, **kwargs):
    from types import SimpleNamespace

    class _MutableCS(SimpleNamespace):
      def __getattr__(self, name):
        if name == "wheelSpeeds":
          wheels = _CarStateShimFactory.WheelSpeeds()
          setattr(self, name, wheels)
          return wheels
        if name == "cruiseState":
          cs = _CarStateShimFactory.CruiseState()
          setattr(self, name, cs)
          return cs
        if name == "buttonEvents":
          lst = []
          setattr(self, name, lst)
          return lst
        setattr(self, name, 0)
        return 0

      def as_reader(self):
        return self

    obj = _MutableCS()
    obj.buttonEvents = []
    obj.wheelSpeeds = _CarStateShimFactory.WheelSpeeds()
    obj.cruiseState = _CarStateShimFactory.CruiseState()
    for k, v in kwargs.items():
      setattr(obj, k, v)
    return obj


# Public CarState shim
CarState = _CarStateShimFactory()
# RadarData shim for mutable usage in radar interfaces
_CapnpRadarData = car.RadarData


class _RadarDataShimFactory:
  class Error:
    def __init__(self):
      self.canError = False
      self.radarFault = False
      self.wrongConfig = False
      self.radarUnavailableTemporary = False

  class RadarPoint:
    def __init__(self):
      self.trackId = 0
      self.dRel = 0.0
      self.yRel = 0.0
      self.vRel = 0.0
      self.aRel = float("nan")
      self.yvRel = float("nan")
      self.measured = False

  def __call__(self, **kwargs):
    from types import SimpleNamespace

    class _MutableRD(SimpleNamespace):
      def __getattr__(self, name):
        if name == "errors":
          e = _RadarDataShimFactory.Error()
          setattr(self, name, e)
          return e
        if name == "points":
          lst = []
          setattr(self, name, lst)
          return lst
        setattr(self, name, 0)
        return 0

      def as_reader(self):
        return self

    obj = _MutableRD()
    obj.errors = _RadarDataShimFactory.Error()
    obj.points = []
    for k, v in kwargs.items():
      setattr(obj, k, v)
    return obj


RadarData = _RadarDataShimFactory()
# Keep original capnpy CarParams for types/nested classes
_CapnpCarParams = car.CarParams

# Provide pycapnp-like enum schema access used elsewhere (e.g., Ecu.schema.enumerants)
try:
  from types import SimpleNamespace as _SimpleNS

  # Replace capnpy Ecu with a serializable Python IntEnum to play nicely with pytest-xdist.
  _members = getattr(_CapnpCarParams.Ecu, "__members__", ())
  if _members:
    EcuInts = type("Ecu", (), {})
    for name in _members:
      setattr(EcuInts, name, int(getattr(_CapnpCarParams.Ecu, name)))
    # Install lightweight mapping for name->int
    _CapnpCarParams.Ecu = EcuInts  # type: ignore[attr-defined]
    _CapnpCarParams.Ecu.schema = _SimpleNS(enumerants={name: getattr(EcuInts, name) for name in _members})
  # Align union type naming with pycapnp expectations
  if hasattr(_CapnpCarParams, "Lateraltuning") and not hasattr(_CapnpCarParams, "LateralTuning"):
    _CapnpCarParams.LateralTuning = _CapnpCarParams.Lateraltuning
except Exception:
  pass


class _CarParamsShim:
  """Minimal mutable shim to emulate pycapnp CarParams construction.
  - Callable to return a plain, mutable object for attribute assignment.
  - Exposes nested types/enums from capnpy-generated class for compatibility.
  """

  # attach nested classes/enums from capnpy CarParams
  for _name in dir(_CapnpCarParams):
    if _name.startswith("_"):
      continue
    _attr = getattr(_CapnpCarParams, _name)
    if isinstance(_attr, type):
      locals()[_name] = _attr
  # ensure union alias exists
  if hasattr(_CapnpCarParams, "LateralTuning"):
    LateralTuning = _CapnpCarParams.LateralTuning

  def __call__(self, **kwargs):
    from types import SimpleNamespace

    class _MutableCP(SimpleNamespace):
      # Provide 0/False defaults for unknown attrs commonly used numerically/bitwise
      def __getattr__(self, name):
        if name in {"safetyConfigs", "carFw"}:
          v = []
          setattr(self, name, v)
          return v
        if name in {"brand", "carVin", "carFingerprint"}:
          setattr(self, name, "")
          return ""
        if name in {"steerControlType"}:
          return _CapnpCarParams.SteerControlType.torque
        # numeric/boolean default
        setattr(self, name, 0)
        return 0

      def as_reader(self):
        return self

    obj = _MutableCP()
    # Pre-initialize commonly-mutated nested structs as mutable namespaces
    obj.longitudinalTuning = SimpleNamespace(kpBP=[], kpV=[], kiBP=[], kiV=[], kf=0.0)
    obj.lateralParams = SimpleNamespace(torqueBP=[], torqueV=[])

    class _UnionLateralTuning(SimpleNamespace):
      def __init__(self):
        super().__init__(
          pid=SimpleNamespace(kpBP=[], kpV=[], kiBP=[], kiV=[], kf=0.0),
          torque=SimpleNamespace(kp=0.0, ki=0.0, friction=0.0, kf=0.0, steeringAngleDeadzoneDeg=0.0, latAccelFactor=0.0, latAccelOffset=0.0),
        )
        self._which = "pid"

      def init(self, which: str):
        self._which = which

      def which(self) -> str:
        return self._which

    obj.lateralTuning = _UnionLateralTuning()
    # Lists/fields and common scalars
    obj.safetyConfigs = []
    obj.carFw = []
    obj.brand = ""
    obj.carVin = ""
    obj.carFingerprint = ""
    obj.fuzzyFingerprint = False
    obj.flags = 0
    # set provided kwargs
    for k, v in kwargs.items():
      setattr(obj, k, v)
    return obj

  # Provide mutable factory for SafetyConfig used by helper
  def SafetyConfig(self, **kwargs):  # type: ignore[override]
    from types import SimpleNamespace

    return SimpleNamespace(safetyModel=kwargs.get("safetyModel"), safetyParam=kwargs.get("safetyParam", 0))


# Public export used throughout the repo
CarParams = _CarParamsShim()


# Patch capnpy repr to tolerate non-UTF8 bytes in Data/Text fields when printing during tests
def _safe_decode_maybe(s):
  if s is None:
    return None
  if isinstance(s, (bytes, bytearray)):
    try:
      return s.decode("utf-8")
    except Exception:
      return s.decode("utf-8", "backslashreplace")
  return s


try:
  _capnpy_util.decode_maybe = _safe_decode_maybe  # type: ignore[assignment]
except Exception:
  pass


# Provide pycapnp-like as_reader on top-level CarControl for tests
class _CarControlShimFactory:
  class Actuators:
    class LongControlState:
      off = 0
      pid = 1
      stopping = 2
      starting = 3

    def __init__(self):
      self.steer = 0.0
      self.torque = 0.0
      self.accel = 0.0
      self.curvature = 0.0
      self.torqueOutputCan = 0
      self.steeringAngleDeg = 0.0
      self.longControlState = _CarControlShimFactory.Actuators.LongControlState.off

    def as_builder(self):
      # Return a shallow copy that is still mutable
      b = _CarControlShimFactory.Actuators()
      b.steer = self.steer
      b.torque = self.torque
      b.accel = self.accel
      b.longControlState = self.longControlState
      return b

  class HUDControl:
    class VisualAlert:
      none = 0
      fcw = 1
      steerRequired = 2
      brakePressed = 3
      wrongGear = 4
      seatbeltUnbuckled = 5
      speedTooHigh = 6
      ldw = 7

    class AudibleAlert:
      none = 0
      engage = 1
      disengage = 2
      refuse = 3
      warningSoft = 4
      warningImmediate = 5
      prompt = 6
      promptRepeat = 7
      promptDistracted = 8

  def __call__(self, **kwargs):
    from types import SimpleNamespace

    class _MutableCC(SimpleNamespace):
      def __getattr__(self, name):
        if name == "actuators":
          act = _CarControlShimFactory.Actuators()
          setattr(self, name, act)
          return act
        if name == "orientationNED":
          arr = []
          setattr(self, name, arr)
          return arr
        if name == "hudControl":
          hc = SimpleNamespace(
            speedVisible=False,
            setSpeed=0.0,
            lanesVisible=False,
            leadVisible=False,
            visualAlert=_CarControlShimFactory.HUDControl.VisualAlert.none,
            rightLaneVisible=False,
            leftLaneVisible=False,
            rightLaneDepart=False,
            leftLaneDepart=False,
            leadDistanceBars=0,
            audibleAlert=_CarControlShimFactory.HUDControl.AudibleAlert.none,
          )
          setattr(self, name, hc)
          return hc
        if name == "cruiseControl":
          cc = SimpleNamespace(cancel=False, resume=False, override=False)
          setattr(self, name, cc)
          return cc
        setattr(self, name, False)
        return False

      def as_reader(self):
        return self

    obj = _MutableCC()
    obj.enabled = False
    obj.latActive = False
    obj.longActive = False
    obj.actuators = _CarControlShimFactory.Actuators()
    for k, v in kwargs.items():
      setattr(obj, k, v)
    return obj


# Public CarControl shim
CarControl = _CarControlShimFactory()

CarStateT = Any
RadarDataT = Any
CarControlT = Any
CarParamsT = Any
