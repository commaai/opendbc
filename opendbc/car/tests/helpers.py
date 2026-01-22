from opendbc.car.can_definitions import CanData


def fingerprint_on_log(msgs: LogIterable, CP: car.CarParams) -> tuple[str, dict[int, dict[int, int]]]:
  # openpilot now uses match_fw_to_car(..., allow_fuzzy=False)
  # however, this means older routes without brand logging won't fingerprint
  exact_fw_match = match_fw_to_car_exact(build_fw_dict(CP.carFw))
  platform_from_fw = list(exact_fw_match)[0] if len(exact_fw_match) == 1 else None

  can_msgs = ([CanData(can.address, can.dat, can.src) for can in m.can] for m in msgs if m.which() == 'can')
  def can_recv(wait_for_one: bool = False) -> list[list[CanData]]:
    return [next(can_msgs, [])]

  platform_from_can, can_fingerprints = can_fingerprint(can_recv)
  offline_fingerprint = platform_from_fw or platform_from_can or MIGRATION.get(CP.carFingerprint, CP.carFingerprint)
  return offline_fingerprint, can_fingerprints

def get_online_car_params(msgs: LogIterable, migrate_platform: bool = False) -> Optional[car.CarParams]:
  CP = next((m.as_builder().carParams for m in msgs if m.which() == "carParams"), None)
  if CP is None:
    return None

  if migrate_platform:
    offline_fingerprint, _ = fingerprint_on_log(msgs, CP)
    CP.carFingerprint = offline_fingerprint

  return CP
