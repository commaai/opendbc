import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.car_helpers import interfaces
from opendbc.car.toyota.support_groups import ToyotaAccGroup, ToyotaCruiseGroup, ToyotaLateralGroup, ToyotaPtGroup, ToyotaSupportEvidence, \
                                                       ToyotaSupportGroups, get_pt_dbc, resolve_from_evidence, resolve_manifest_candidates
from opendbc.car.toyota.values import CAR, EPS_SCALE


class TestToyotaSupportGroups(unittest.TestCase):
  def setUp(self):
    self.fingerprint = gen_empty_fingerprint()

  def groups(self, candidate):
    return ToyotaSupportGroups.from_platform(candidate, self.fingerprint, [])

  def test_identity_ambiguity_can_resolve_one_protocol(self):
    candidates = {CAR.TOYOTA_CAMRY_TSS2, CAR.TOYOTA_COROLLA_TSS2, CAR.TOYOTA_HIGHLANDER_TSS2}
    groups, identities = resolve_manifest_candidates(candidates, self.fingerprint, [])
    self.assertIsNotNone(groups)
    self.assertEqual(identities, candidates)

  def test_real_protocol_change_remains_ambiguous(self):
    candidates = {CAR.TOYOTA_RAV4_TSS2_2022, CAR.TOYOTA_RAV4_TSS2_2023}
    groups, _ = resolve_manifest_candidates(candidates, self.fingerprint, [])
    self.assertIsNone(groups)
    self.assertEqual(self.groups(CAR.TOYOTA_RAV4_TSS2_2022).lateral, ToyotaLateralGroup.LKA_TORQUE)
    self.assertEqual(self.groups(CAR.TOYOTA_RAV4_TSS2_2023).lateral, ToyotaLateralGroup.LTA_ANGLE)

  def test_generic_toyota_dynamics(self):
    expected = None
    for candidate in CAR:
      CP = interfaces[candidate].get_params(candidate, self.fingerprint, [], False, False, False)
      dynamics = (CP.mass, CP.wheelbase, CP.steerRatio, CP.tireStiffnessFactor, CP.centerToFront)
      expected = dynamics if expected is None else expected
      self.assertEqual(dynamics, expected)

  def test_carstate_dbc_comes_from_group(self):
    for candidate in CAR:
      CP = interfaces[candidate].get_params(candidate, self.fingerprint, [], False, False, False)
      self.assertEqual(interfaces[candidate].CarState.get_can_parsers(CP)["pt"].dbc_name, get_pt_dbc(CP))

  def test_eps_scale_is_preserved_as_open_problem(self):
    for candidate in CAR:
      CP = interfaces[candidate].get_params(candidate, self.fingerprint, [], False, False, False)
      self.assertEqual(CP.safetyConfigs[-1].safetyParam & 0xFF, EPS_SCALE[candidate])

  def test_live_resolution_does_not_need_identity(self):
    resolution = resolve_from_evidence(ToyotaSupportEvidence(
      pt=ToyotaPtGroup.NO_DSU,
      cruise=ToyotaCruiseGroup.PCM,
      acc=ToyotaAccGroup.CAMERA,
      stock_lta_active=False,
      eps_torque_scale=73,
      hybrid=False,
      has_lkas_hud=True,
      stop_and_go=True,
    ))
    self.assertFalse(resolution.missing)
    self.assertEqual(resolution.groups.lateral, ToyotaLateralGroup.LKA_TORQUE)

  def test_live_resolution_fails_closed_on_eps_scale(self):
    resolution = resolve_from_evidence(ToyotaSupportEvidence(
      pt=ToyotaPtGroup.NO_DSU,
      cruise=ToyotaCruiseGroup.PCM,
      acc=ToyotaAccGroup.CAMERA,
      stock_lta_active=False,
      hybrid=False,
      has_lkas_hud=True,
      stop_and_go=True,
    ))
    self.assertIsNone(resolution.groups)
    self.assertEqual(resolution.missing, {"eps_torque_scale"})
