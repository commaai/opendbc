"""Signs CONTROL_REQUEST (0x08A) through the EPS-resident SecOC signer."""
from collections import OrderedDict
from dataclasses import dataclass

from opendbc.car import DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.values import CarControllerParams

# The camera harness is repinned so the vehicle network is on bus 0 and the FRC on bus 2
TSS3_CHASSIS_BUS = 0
TSS3_AUX_BUS = 1
TSS3_SOURCE_BUS = 2

SIGNER_SID = 0xC9
SIGNER_SEQUENCE_MAX = 0xFF
# The signer answers in 17 ms (p50) to 32 ms (p99). Four requests in flight hide that at 100 Hz,
# more only add latency between computing a request and publishing it.
SIGNER_MAX_PENDING = 4
SIGNER_TIMEOUT_NS = 90_000_000
REQUEST_INTERVAL_NS = int(DT_CTRL * 1e9)

# The VMC in the brake ECU sets CONTROL_RESULT.REQUEST_LOSS ~90 ms after the last valid CONTROL_REQUEST and latches
# a cruise fault until restart if it persists for ~1 s. Panda forwards the FRC's CONTROL_REQUEST again if openpilot's
# stops for 100 ms, and it restarts the angle rate limit from the measured angle after a 100 ms gap in requests.
CONTROL_REQUEST_TIMEOUT_NS = 100_000_000


def target_angle_deg_to_raw(angle_deg: float) -> int:
  return round(angle_deg / CarControllerParams.TSS3_TARGET_ANGLE_SCALE_DEG)


@dataclass
class SignRequest:
  values: dict
  queued_ns: int
  trailer: dict | None = None


class SignerTransport:
  """Signs one CONTROL_REQUEST per controller frame and publishes the signed result.

  A few requests are kept in flight to hide the signer latency. Panda checks each request when it is
  sent to the signer, and only lets approved requests be published.
  """

  def __init__(self, packer, stock_longitudinal: bool):
    self.packer = packer
    self.stock_longitudinal = stock_longitudinal
    self.pending: OrderedDict[int, SignRequest] = OrderedDict()  # by signer sequence, in request order
    self.next_signer_sequence = 1
    self.next_request_sequence = 0
    self.last_request_ns = 0
    self.request_rejected = False
    self.active = False  # panda publishes openpilot's CONTROL_REQUEST instead of the FRC's
    self.started_ns = 0
    self.last_publish_ns = 0
    self.last_publish_sequence: int | None = None

  def _release(self, sends: list[CanData], reason: str | None = None) -> None:
    if reason is not None:
      carlog.error(f"Toyota TSS3 signer transport restart: {reason}")
    if self.active:
      sends.append(toyotacan.create_tss3_signer_arm(self.packer, TSS3_AUX_BUS, False))
    self.active = False
    self.pending.clear()
    self.started_ns = 0
    self.last_publish_ns = 0
    self.last_publish_sequence = None

  def receive(self, CS, enabled: bool, now_ns: int) -> list[CanData]:
    """Process the signer responses and panda rejections from this frame's CAN."""
    sends: list[CanData] = []
    self.request_rejected |= CS.tss3_signer_request_rejected
    if CS.tss3_control_request_rejected and self.active:
      # panda forwards the FRC's CONTROL_REQUEST again, re-arm with the next signed request
      self._release(sends, "control_request_rejected")

    for response in CS.tss3_signer_responses:
      seq = int(response["SIGNER_SEQUENCE"])
      if response["SERVICE_ID"] != SIGNER_SID or int(response["SIGNER_SEQUENCE_INVERTED"]) != seq ^ 0xFF or seq not in self.pending:
        continue

      # the signer answers in order, so earlier unanswered requests were lost
      for older_seq in list(self.pending):
        if older_seq == seq:
          break
        if self.pending[older_seq].trailer is None:
          del self.pending[older_seq]

      request = self.pending[seq]
      if response["STATUS"] != 0:
        carlog.warning(f"Toyota TSS3 signer status {int(response['STATUS'])}")
        del self.pending[seq]
      elif now_ns - request.queued_ns > SIGNER_TIMEOUT_NS:
        del self.pending[seq]
      else:
        request.trailer = {s: response[s] for s in ("MSG_CNT_LOWER", "RESET_FLAG", "AUTHENTICATOR")}

    if not enabled or not CS.out.canValid:
      self._release(sends)
      self.next_request_sequence = 0
    elif self.started_ns and now_ns - (self.last_publish_ns or self.started_ns) > SIGNER_TIMEOUT_NS:
      self._release(sends, "signer_timeout")
    return sends

  def _publish_ready(self, now_ns: int) -> bool:
    request = next(iter(self.pending.values()), None)
    if request is None or request.trailer is None:
      return False
    if self.last_publish_sequence is not None:
      # keep 10 ms per request after a lost response, don't compress steps
      steps = (request.values["REQUEST_SEQUENCE"] - self.last_publish_sequence) & 0x3F
      if steps > 1 and now_ns - self.last_publish_ns < steps * REQUEST_INTERVAL_NS:
        return False
    return True

  def angle_reference_reset(self, now_ns: int) -> bool:
    """Whether panda restarted its angle rate limit from the measured angle, the controller should do the same."""
    reset = self.request_rejected or now_ns - self.last_request_ns > CONTROL_REQUEST_TIMEOUT_NS
    self.request_rejected = False
    return reset

  def request_due(self, CS, enabled: bool, now_ns: int) -> bool:
    """Whether send() will sign a new request this frame."""
    if not enabled or not CS.out.canValid:
      return False
    return len(self.pending) - int(self._publish_ready(now_ns)) < SIGNER_MAX_PENDING

  def send(self, CS, enabled: bool, lat_active: bool, angle_deg: float, long_active: bool, accel: float,
           now_ns: int) -> list[CanData]:
    """Publish the next signed request and sign the latest one, call after receive()."""
    sends: list[CanData] = []
    if not enabled or not CS.out.canValid:
      return sends
    if self.started_ns == 0:
      self.started_ns = now_ns

    # publish at most one signed request per frame
    if self._publish_ready(now_ns):
      _, request = self.pending.popitem(last=False)
      if not self.active:
        sends.append(toyotacan.create_tss3_signer_arm(self.packer, TSS3_AUX_BUS, True))
        self.active = True
      sends.append(self.packer.make_can_msg("CONTROL_REQUEST", TSS3_CHASSIS_BUS, request.values | request.trailer))
      self.last_publish_ns = now_ns
      self.last_publish_sequence = request.values["REQUEST_SEQUENCE"]

    # then sign the latest request
    if len(self.pending) < SIGNER_MAX_PENDING:
      stock_request = CS.tss3_stock_control_request if self.stock_longitudinal else None
      values = toyotacan.create_tss3_control_request_values(stock_request, lat_active, target_angle_deg_to_raw(angle_deg),
                                                            long_active, accel, CS.out.vCruise, self.next_request_sequence)
      application = self.packer.make_can_msg("CONTROL_REQUEST", TSS3_CHASSIS_BUS, values)[1][:28]
      seq = self.next_signer_sequence
      self.next_signer_sequence = seq % SIGNER_SEQUENCE_MAX + 1
      self.pending.pop(seq, None)
      self.pending[seq] = SignRequest(values, now_ns)
      self.next_request_sequence = (self.next_request_sequence + 1) & 0x3F
      self.last_request_ns = now_ns
      sends.extend(toyotacan.create_tss3_signer_requests(self.packer, TSS3_CHASSIS_BUS, seq, application))

    return sends
