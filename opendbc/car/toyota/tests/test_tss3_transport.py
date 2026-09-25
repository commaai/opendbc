import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car.toyota.tss3 import SIGNER_MAX_PENDING, SIGNER_TIMEOUT_NS, TSS3_AUX_BUS, TSS3_CHASSIS_BUS, SignerTransport

DBC = "toyota_tss3_pt_generated"
ARM, RELEASE = bytes.fromhex("07c9a80100000000"), bytes.fromhex("07c9a80000000000")
TRAILER = {"MSG_CNT_LOWER": 1, "RESET_FLAG": 2, "AUTHENTICATOR": 0x1234567}
MS = 1_000_000


def state(responses=(), stock=None, can_valid=True, request_rejected=False, control_request_rejected=False):
  return SimpleNamespace(out=SimpleNamespace(canValid=can_valid, vCruise=70.0), tss3_signer_responses=list(responses),
                         tss3_stock_control_request=stock, tss3_signer_request_rejected=request_rejected,
                         tss3_control_request_rejected=control_request_rejected)


def response(seq, status=0):
  return {"SERVICE_ID": 0xC9, "SIGNER_SEQUENCE": seq, "STATUS": status, "SIGNER_SEQUENCE_INVERTED": seq ^ 0xFF, **TRAILER}


def signer_requests(sends):
  return [(dat, bus) for addr, dat, bus in sends if addr == 0x777 and dat[0] >= 0x80]


def published(sends):
  return [dat for addr, dat, _ in sends if addr == 0x08A]


class TestSignerTransport(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker(DBC)
    self.signer = SignerTransport(self.packer, stock_longitudinal=False)
    self.t = 1_000 * MS

  def step(self, CS=None, enabled=True, angle=1.0, accel=0.5):
    CS = CS or state()
    sends = self.signer.receive(CS, enabled, self.t)
    sends += self.signer.send(CS, enabled, True, angle, True, accel, self.t)
    self.t += 10 * MS
    return sends

  def test_request_fragments_and_signed_publication(self):
    sends = self.step()
    fragments = signer_requests(sends)
    self.assertEqual([dat[0] for dat, _ in fragments], [0x81, 0x90, 0xA1, 0xB0])
    self.assertTrue(all(bus == TSS3_CHASSIS_BUS for _, bus in fragments))
    application = b"".join(dat[1:] for dat, _ in fragments)

    sends = self.step(state([response(1)]))
    self.assertEqual(sends[0], (0x777, ARM, TSS3_AUX_BUS))
    frame = published(sends)[0]
    self.assertEqual(frame[:28], application)
    parser = CANParser(DBC, [("CONTROL_REQUEST", float("nan"))], 0)
    parser.update([(0, [(0x08A, frame, 0)])])
    self.assertEqual({s: parser.vl["CONTROL_REQUEST"][s] for s in TRAILER}, TRAILER)
    self.assertEqual(parser.vl["CONTROL_REQUEST"]["LATERAL_REQUEST_ID"], 11)
    self.assertAlmostEqual(parser.vl["CONTROL_REQUEST"]["LONGITUDINAL_REQUEST_ACCEL_UPPER"], 0.5)

  def test_stock_longitudinal_only_replaces_lateral_fields_and_sequence(self):
    stock_frame = bytes.fromhex("0000000880002d47fe462afe467fff007fffff35c000100064003c005db7797f")
    parser = CANParser(DBC, [("CONTROL_REQUEST", float("nan"))], 2)
    parser.update([(0, [(0x08A, stock_frame, 2)])])
    self.signer = SignerTransport(self.packer, stock_longitudinal=True)
    application = b"".join(dat[1:] for dat, _ in signer_requests(self.step(state(stock=dict(parser.vl["CONTROL_REQUEST"])))))
    for i, (ours, stock) in enumerate(zip(application, stock_frame[:28], strict=True)):
      mask = {18: 0, 19: 0, 24: 0, 25: 0, 21: 0xC0, 26: 0xC0}.get(i, 0xFF)  # as panda checks it
      self.assertEqual(ours & mask, stock & mask, i)

  def test_pipeline_publishes_in_order_at_100hz(self):
    for _ in range(SIGNER_MAX_PENDING + 2):
      self.step()
    self.assertEqual(len(self.signer.pending), SIGNER_MAX_PENDING)

    sequences = []
    for seq in range(1, 9):
      frames = published(self.step(state([response(seq)])))
      self.assertEqual(len(frames), 1)
      sequences.append(frames[0][26] & 0x3F)
    self.assertEqual(sequences, list(range(8)))

  def test_lost_response_drops_earlier_requests_and_keeps_request_timing(self):
    self.step()
    self.step()
    # response for the second request, the first was lost
    self.assertEqual(published(self.step(state([response(2)])))[0][26] & 0x3F, 1)
    self.assertNotIn(1, self.signer.pending)

    self.step(state([response(3)]))
    # request 4 was lost, request 5 is two steps after the last published and waits 20 ms
    self.assertEqual(published(self.step(state([response(5)]))), [])

  def test_signer_error_and_invalid_responses_are_skipped(self):
    self.step()
    bad = dict(response(1), SIGNER_SEQUENCE_INVERTED=0)
    self.assertEqual(published(self.step(state([bad, response(99)]))), [])
    assert 1 in self.signer.pending
    self.step(state([response(1, status=5)]))
    self.assertNotIn(1, self.signer.pending)

  def test_release_on_disable_invalid_can_and_timeout(self):
    for release in ({"enabled": False}, {"CS": state(can_valid=False)}):
      with self.subTest(**{k: str(v) for k, v in release.items()}):
        self.setUp()
        self.step()
        self.step(state([response(1)]))
        sends = self.step(**release)
        assert (0x777, RELEASE, TSS3_AUX_BUS) in sends
        self.assertEqual((self.signer.active, len(self.signer.pending)), (False, 0))

    self.setUp()
    self.step()
    self.step(state([response(1)]))
    for _ in range(SIGNER_TIMEOUT_NS // (10 * MS) + 1):
      sends = self.step()
    assert (0x777, RELEASE, TSS3_AUX_BUS) in sends

  def test_rejected_publication_releases_and_rearms(self):
    self.step()
    self.step()
    self.step(state([response(1)]))
    sends = self.step(state([response(2)], control_request_rejected=True))
    self.assertEqual([dat for addr, dat, _ in sends if addr == 0x777 and dat[0] == 0x07], [RELEASE])
    self.step()
    assert (0x777, ARM, TSS3_AUX_BUS) in self.step(state([response(4)]))

  def test_angle_reference_reset(self):
    self.assertTrue(self.signer.angle_reference_reset(self.t))
    self.step()
    self.assertFalse(self.signer.angle_reference_reset(self.t))
    self.step(state(request_rejected=True))
    self.assertTrue(self.signer.angle_reference_reset(self.t))
    self.assertFalse(self.signer.angle_reference_reset(self.t))
    self.assertTrue(self.signer.angle_reference_reset(self.t + 200 * MS))


if __name__ == "__main__":
  unittest.main()
