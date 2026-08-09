from dataclasses import dataclass
from enum import StrEnum

from opendbc.car import Bus, structs


class ToyotaPtGroup(StrEnum):
  NEW_MC = "new_mc"
  TNGA_K = "tnga_k"
  NO_DSU = "no_dsu"
  SECOC = "secoc"


class ToyotaCruiseGroup(StrEnum):
  PCM = "pcm"
  DSU = "dsu"


class ToyotaAccGroup(StrEnum):
  POWERTRAIN = "powertrain"
  CAMERA = "camera"
  RADAR = "radar"


class ToyotaLateralGroup(StrEnum):
  LKA_TORQUE = "lka_torque"
  LTA_ANGLE = "lta_angle"


class ToyotaLongitudinalGroup(StrEnum):
  STOCK = "stock"
  CAMERA_ACC = "camera_acc"
  RADAR_ACC = "radar_acc"
  SECOC_ACC = "secoc_acc"


@dataclass(frozen=True)
class ToyotaSupportGroups:
  pt: ToyotaPtGroup
  cruise: ToyotaCruiseGroup
  acc: ToyotaAccGroup
  lateral: ToyotaLateralGroup
  longitudinal: ToyotaLongitudinalGroup
  eps_torque_scale: int
  tss2: bool
  secoc: bool
  hybrid: bool
  has_lkas_hud: bool
  stop_and_go: bool
  raised_accel_limit: bool
  wheel_speed_factor: float

  @property
  def protocol_key(self) -> tuple:
    """The protocol composition, intentionally excluding identity."""
    return (self.pt, self.cruise, self.acc, self.lateral, self.longitudinal,
            self.eps_torque_scale, self.tss2, self.secoc, self.hybrid,
            self.has_lkas_hud, self.stop_and_go, self.raised_accel_limit,
            self.wheel_speed_factor)

  @classmethod
  def from_platform(cls, candidate: str, fingerprint: dict[int, dict[int, int]],
                    car_fw: list[structs.CarParams.CarFw]) -> 'ToyotaSupportGroups':
    """Migration adapter from a platform identity to independently consumed groups.

    This is deliberately the only place where the prototype translates the old
    platform bundle. A live resolver can replace it without changing CarState or
    CarController.
    """
    from opendbc.car.toyota.values import ANGLE_CONTROL_CAR, CAR, DBC, EPS_SCALE, RADAR_ACC_CAR, SECOC_CAR, TSS2_CAR, UNSUPPORTED_DSU_CAR

    pt_dbc = DBC[candidate][Bus.pt]
    pt = {
      "toyota_new_mc_pt_generated": ToyotaPtGroup.NEW_MC,
      "toyota_tnga_k_pt_generated": ToyotaPtGroup.TNGA_K,
      "toyota_nodsu_pt_generated": ToyotaPtGroup.NO_DSU,
      "toyota_secoc_pt_generated": ToyotaPtGroup.SECOC,
    }[pt_dbc]

    tss2 = candidate in TSS2_CAR
    secoc = candidate in SECOC_CAR
    radar_acc = candidate in RADAR_ACC_CAR
    if radar_acc:
      acc = ToyotaAccGroup.RADAR
    elif tss2:
      acc = ToyotaAccGroup.CAMERA
    else:
      acc = ToyotaAccGroup.POWERTRAIN

    if secoc:
      longitudinal = ToyotaLongitudinalGroup.SECOC_ACC
    elif radar_acc:
      longitudinal = ToyotaLongitudinalGroup.RADAR_ACC
    elif tss2:
      longitudinal = ToyotaLongitudinalGroup.CAMERA_ACC
    else:
      longitudinal = ToyotaLongitudinalGroup.STOCK

    stop_and_go = tss2
    if candidate in (CAR.TOYOTA_PRIUS, CAR.LEXUS_RX, CAR.LEXUS_RX_TSS2):
      stop_and_go = True
    elif candidate in (CAR.TOYOTA_AVALON, CAR.TOYOTA_AVALON_2019, CAR.TOYOTA_AVALON_TSS2):
      stop_and_go = candidate != CAR.TOYOTA_AVALON
    elif candidate in (CAR.TOYOTA_CHR, CAR.TOYOTA_CAMRY, CAR.TOYOTA_SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_LS, CAR.LEXUS_NX):
      stop_and_go = True

    return cls(
      pt=pt,
      cruise=ToyotaCruiseGroup.DSU if candidate in UNSUPPORTED_DSU_CAR else ToyotaCruiseGroup.PCM,
      acc=acc,
      lateral=ToyotaLateralGroup.LTA_ANGLE if candidate in ANGLE_CONTROL_CAR else ToyotaLateralGroup.LKA_TORQUE,
      longitudinal=longitudinal,
      eps_torque_scale=EPS_SCALE[candidate],
      tss2=tss2,
      secoc=secoc,
      hybrid=any(fw.ecu == structs.CarParams.Ecu.hybrid for fw in car_fw),
      has_lkas_hud=candidate != CAR.TOYOTA_PRIUS_V,
      stop_and_go=stop_and_go,
      raised_accel_limit=tss2,
      wheel_speed_factor=1.035 if candidate in (CAR.LEXUS_RX, CAR.LEXUS_RX_TSS2) else 1.0,
    )


@dataclass(frozen=True)
class ToyotaSupportEvidence:
  """Facts produced by parallel readers and stock-message observation."""
  pt: ToyotaPtGroup | None = None
  cruise: ToyotaCruiseGroup | None = None
  acc: ToyotaAccGroup | None = None
  stock_lta_active: bool | None = None
  eps_torque_scale: int | None = None
  hybrid: bool | None = None
  has_lkas_hud: bool | None = None
  stop_and_go: bool | None = None
  wheel_speed_factor: float | None = None


@dataclass(frozen=True)
class ToyotaSupportResolution:
  groups: ToyotaSupportGroups | None
  missing: frozenset[str]


def resolve_from_evidence(evidence: ToyotaSupportEvidence) -> ToyotaSupportResolution:
  """Resolve support groups without a platform identity, failing on unknowns."""
  required = {
    "pt": evidence.pt,
    "cruise": evidence.cruise,
    "acc": evidence.acc,
    "stock_lta_active": evidence.stock_lta_active,
    "eps_torque_scale": evidence.eps_torque_scale,
    "hybrid": evidence.hybrid,
    "has_lkas_hud": evidence.has_lkas_hud,
    "stop_and_go": evidence.stop_and_go,
  }
  missing = frozenset(name for name, value in required.items() if value is None)
  if missing:
    return ToyotaSupportResolution(None, missing)

  assert evidence.pt is not None
  assert evidence.cruise is not None
  assert evidence.acc is not None
  assert evidence.stock_lta_active is not None
  assert evidence.eps_torque_scale is not None
  assert evidence.hybrid is not None
  assert evidence.has_lkas_hud is not None
  assert evidence.stop_and_go is not None

  tss2 = evidence.pt in (ToyotaPtGroup.NO_DSU, ToyotaPtGroup.SECOC)
  secoc = evidence.pt == ToyotaPtGroup.SECOC
  if secoc:
    longitudinal = ToyotaLongitudinalGroup.SECOC_ACC
  elif evidence.acc == ToyotaAccGroup.RADAR:
    longitudinal = ToyotaLongitudinalGroup.RADAR_ACC
  elif evidence.acc == ToyotaAccGroup.CAMERA:
    longitudinal = ToyotaLongitudinalGroup.CAMERA_ACC
  else:
    longitudinal = ToyotaLongitudinalGroup.STOCK

  groups = ToyotaSupportGroups(
    pt=evidence.pt,
    cruise=evidence.cruise,
    acc=evidence.acc,
    lateral=ToyotaLateralGroup.LTA_ANGLE if evidence.stock_lta_active else ToyotaLateralGroup.LKA_TORQUE,
    longitudinal=longitudinal,
    eps_torque_scale=evidence.eps_torque_scale,
    tss2=tss2,
    secoc=secoc,
    hybrid=evidence.hybrid,
    has_lkas_hud=evidence.has_lkas_hud,
    stop_and_go=evidence.stop_and_go,
    raised_accel_limit=tss2,
    wheel_speed_factor=evidence.wheel_speed_factor if evidence.wheel_speed_factor is not None else 1.0,
  )
  return ToyotaSupportResolution(groups, frozenset())


TOYOTA_PT_DBC = {
  ToyotaPtGroup.NEW_MC: "toyota_new_mc_pt_generated",
  ToyotaPtGroup.TNGA_K: "toyota_tnga_k_pt_generated",
  ToyotaPtGroup.NO_DSU: "toyota_nodsu_pt_generated",
  ToyotaPtGroup.SECOC: "toyota_secoc_pt_generated",
}

PT_GROUP_FLAGS = {
  ToyotaPtGroup.NEW_MC: 1 << 12,
  ToyotaPtGroup.TNGA_K: 1 << 13,
  ToyotaPtGroup.NO_DSU: 1 << 14,
  ToyotaPtGroup.SECOC: 1 << 15,
}


def apply_support_groups(CP: structs.CarParams, groups: ToyotaSupportGroups) -> None:
  """Encode resolved groups in CarParams without retaining platform dispatch."""
  from opendbc.car.toyota.values import ToyotaFlags

  CP.flags = int(CP.flags | PT_GROUP_FLAGS[groups.pt])
  if groups.tss2:
    CP.flags = int(CP.flags | ToyotaFlags.TSS2.value | ToyotaFlags.NO_DSU.value)
  if groups.secoc:
    CP.flags = int(CP.flags | ToyotaFlags.SECOC.value)
  if groups.cruise == ToyotaCruiseGroup.DSU:
    CP.flags = int(CP.flags | ToyotaFlags.UNSUPPORTED_DSU.value)
  if groups.lateral == ToyotaLateralGroup.LTA_ANGLE:
    CP.flags = int(CP.flags | ToyotaFlags.ANGLE_CONTROL.value)
  if groups.raised_accel_limit:
    CP.flags = int(CP.flags | ToyotaFlags.RAISED_ACCEL_LIMIT.value)
  if groups.hybrid:
    CP.flags = int(CP.flags | ToyotaFlags.HYBRID.value)
  if groups.acc == ToyotaAccGroup.CAMERA:
    CP.flags = int(CP.flags | ToyotaFlags.ACC_CAMERA.value)
  elif groups.acc == ToyotaAccGroup.RADAR:
    CP.flags = int(CP.flags | ToyotaFlags.ACC_RADAR.value | ToyotaFlags.RADAR_ACC.value)
  if groups.has_lkas_hud:
    CP.flags = int(CP.flags | ToyotaFlags.LKA_HUD.value)


def get_pt_group(CP: structs.CarParams) -> ToyotaPtGroup:
  matches = [group for group, flag in PT_GROUP_FLAGS.items() if CP.flags & flag]
  if len(matches) != 1:
    raise ValueError(f"expected one Toyota PT group, found {matches}")
  return matches[0]


def get_pt_dbc(CP: structs.CarParams) -> str:
  return TOYOTA_PT_DBC[get_pt_group(CP)]


def get_acc_group(CP: structs.CarParams) -> ToyotaAccGroup:
  from opendbc.car.toyota.values import ToyotaFlags

  if CP.flags & ToyotaFlags.ACC_RADAR:
    return ToyotaAccGroup.RADAR
  if CP.flags & ToyotaFlags.ACC_CAMERA:
    return ToyotaAccGroup.CAMERA
  return ToyotaAccGroup.POWERTRAIN


def resolve_manifest_candidates(candidates: set[str], fingerprint: dict[int, dict[int, int]],
                                car_fw: list[structs.CarParams.CarFw]) -> tuple[ToyotaSupportGroups | None, frozenset[str]]:
  """Collapse identity candidates when they imply one support-group composition.

  This is the bridge needed for group fingerprinting: ambiguity between Camry,
  Corolla, and Highlander does not block protocol selection when their complete
  read/write manifests agree. Identity candidates remain available to the
  separate VIN support-policy layer.
  """
  manifests = {ToyotaSupportGroups.from_platform(candidate, fingerprint, car_fw) for candidate in candidates}
  resolved = next(iter(manifests)) if len(manifests) == 1 else None
  return resolved, frozenset(candidates)
