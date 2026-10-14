import unittest

from opendbc.can import CANPacker
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.radar_interface import RadarInterface
from opendbc.car.toyota.values import CAR

RADAR_BUS = 1
EMPTY = {'dist': 0xFFF8 * 0.005, 'lat': 0, 'vrel': 0, 'state': 0, 'new': 0, 'ended': 0}
TRACK = {'dist': 20, 'lat': 1, 'vrel': -1, 'state': 2}


class TestToyotaTSS3Radar(unittest.TestCase):
  def setUp(self):
    CP = CarInterface.get_non_essential_params(CAR.TOYOTA_CAMRY_TSS3)
    CP.radarUnavailable = False
    self.ri = RadarInterface(CP)
    self.packer = CANPacker('toyota_tss3_radar_generated')
    self.t = 1_000_000_000

  def geometry_msg(self, bank, objects=None):
    geo = {}
    for slot in range(8):
      obj = EMPTY | (objects or {}).get(bank * 8 + slot, {})
      geo |= {f'DIST_{slot}': obj['dist'], f'LAT_{slot}': obj['lat']}
    return self.packer.make_can_msg(f'OBJECT_GEOMETRY_{bank}', RADAR_BUS, geo)

  def motion_msg(self, bank, objects=None):
    motion = {}
    for slot in range(8):
      obj = EMPTY | (objects or {}).get(bank * 8 + slot, {})
      motion |= {f'VREL_{slot}': obj['vrel'], f'TRACK_STATE_{slot}': obj['state'],
                 f'NEW_TRACK_{slot}': obj['new'], f'TRACK_ENDED_{slot}': obj['ended']}
    return self.packer.make_can_msg(f'OBJECT_MOTION_{bank}', RADAR_BUS, motion)

  def send(self, msgs):
    rr = self.ri.update([(self.t, list(msgs))])
    self.t += 50_000_000
    return rr

  def update(self, objects=None, geometry=True):
    msgs = []
    for bank in range(3):
      if geometry:
        msgs.append(self.geometry_msg(bank, objects))
      msgs.append(self.motion_msg(bank, objects))
    return self.send(msgs)

  def track_ids(self, objects=None):
    return [pt.trackId for pt in self.update(objects).points]

  def test_geometry_all_slots(self):
    objects = {key: {'dist': 10 + key, 'lat': (key - 12) * 0.2, 'vrel': -key * 0.5, 'state': 2, 'new': 1} for key in range(24)}
    rr = self.update(objects)
    self.assertFalse(rr.errors.canError)
    self.assertEqual(len(rr.points), 24)
    self.assertEqual(len({pt.trackId for pt in rr.points}), 24)
    for key, obj in objects.items():
      pt = self.ri.pts[key]
      self.assertAlmostEqual(pt.dRel, obj['dist'], places=3)
      self.assertAlmostEqual(pt.yRel, obj['lat'], places=3)
      self.assertAlmostEqual(pt.vRel, obj['vrel'], places=3)

  def test_track_lifecycle(self):
    first = self.track_ids({0: TRACK | {'new': 1}})
    self.assertEqual(self.track_ids({0: TRACK}), first)
    replaced = self.track_ids({0: TRACK | {'new': 1}})
    self.assertNotEqual(replaced, first)
    replaced_again = self.track_ids({0: TRACK | {'new': 1, 'ended': 1}})
    self.assertEqual(len(replaced_again), 1)
    self.assertNotEqual(replaced_again, replaced)
    self.assertEqual(self.track_ids({0: TRACK | {'ended': 1}}), [])
    self.assertNotEqual(self.track_ids({0: TRACK}), replaced_again)

  def test_empty_slots(self):
    for empty in ({'state': 0}, {'dist': 0}, {'dist': EMPTY['dist']}):
      with self.subTest(empty=empty):
        first = self.track_ids({3: TRACK})
        self.assertEqual(self.track_ids({3: TRACK | empty}), [])
        self.assertNotEqual(self.track_ids({3: TRACK}), first)
    self.assertEqual(self.track_ids(), [])

  def test_can_error_clears_points(self):
    self.assertEqual(len(self.update({0: TRACK}).points), 1)
    for _ in range(20):
      rr = self.update({0: TRACK}, geometry=False)
    self.assertTrue(rr.errors.canError)
    self.assertEqual(len(rr.points), 0)
    self.assertEqual(self.ri.pts, {})

  def test_missed_geometry(self):
    first = self.track_ids({0: TRACK | {'new': 1}})
    rr = self.send([self.geometry_msg(1), self.geometry_msg(2),
                    self.motion_msg(0, {0: TRACK | {'vrel': -5}}), self.motion_msg(1), self.motion_msg(2)])
    self.assertFalse(rr.errors.canError)
    self.assertEqual([(p.trackId, p.dRel, p.vRel) for p in rr.points], [(first[0], 20, -1)])
    rr = self.update({0: TRACK | {'dist': 25, 'vrel': -5}})
    self.assertEqual([(p.trackId, p.dRel, p.vRel) for p in rr.points], [(first[0], 25, -5)])

  def test_missed_motion_after_new_track(self):
    first = self.track_ids({0: TRACK | {'new': 1}})
    rr = self.send([self.geometry_msg(0, {0: TRACK | {'dist': 25}}), self.geometry_msg(1), self.geometry_msg(2),
                    self.motion_msg(1), self.motion_msg(2)])
    self.assertFalse(rr.errors.canError)
    self.assertEqual([(p.trackId, p.dRel, p.vRel) for p in rr.points], [(first[0], 20, -1)])
    rr = self.update({0: TRACK | {'dist': 25, 'vrel': -5}})
    self.assertEqual([(p.trackId, p.dRel, p.vRel) for p in rr.points], [(first[0], 25, -5)])

  def test_lifecycle_without_geometry_invalidates(self):
    for event in ({'new': 1}, {'ended': 1}, {'state': 0}):
      with self.subTest(event=event):
        first = self.track_ids({0: TRACK})
        rr = self.send([self.geometry_msg(1), self.geometry_msg(2),
                        self.motion_msg(0, {0: TRACK | event}), self.motion_msg(1), self.motion_msg(2)])
        self.assertEqual(len(rr.points), 0)
        recovered = self.track_ids({0: TRACK})
        self.assertEqual(len(recovered), 1)
        self.assertNotEqual(recovered, first)

  def test_split_updates(self):
    first = self.track_ids({0: TRACK | {'new': 1}})
    self.assertIsNone(self.send([self.geometry_msg(0, {0: TRACK | {'dist': 25}})]))
    rr = self.send([self.motion_msg(0, {0: TRACK | {'vrel': -5}}), self.motion_msg(1), self.motion_msg(2)])
    self.assertEqual([(p.trackId, p.dRel, p.vRel) for p in rr.points], [(first[0], 25, -5)])


if __name__ == '__main__':
  unittest.main()
