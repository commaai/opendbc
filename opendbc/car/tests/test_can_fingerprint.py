

class TestCanFingerprint:
  def test_can_fingerprint(self, subtests, car_lib):
    """Tests online fingerprinting function on offline fingerprints"""
    from opendbc.car.fingerprints import _FINGERPRINTS as FINGERPRINTS

    for car_model, fingerprints in FINGERPRINTS.items():
      with subtests.test(car_model=str(car_model)):
        for fingerprint in fingerprints:
          can = [car_lib.CanData(address=address, dat=b'\x00' * length, src=src)
                 for address, length in fingerprint.items() for src in (0, 1)]

          fingerprint_iter = iter([can])

          def can_recv(fingerprint_iter=fingerprint_iter, **kwargs):
            return [next(fingerprint_iter, [])]
          car_fingerprint, finger = car_lib.car_helpers.can_fingerprint(can_recv)
          assert car_fingerprint == car_model
          assert finger[0] == fingerprint
          assert finger[1] == fingerprint
          assert finger[2] == {}

  def test_timing(self, subtests, car_lib):
    from opendbc.car.fingerprints import _FINGERPRINTS as FINGERPRINTS

    # just pick any CAN fingerprinting car
    car_model = "CHEVROLET_BOLT_EUV"
    fingerprint = FINGERPRINTS[car_model][0]

    cases = []

    # case 1 - one match, make sure we keep going for 100 frames
    can = [car_lib.CanData(address=address, dat=b'\x00' * length, src=src)
           for address, length in fingerprint.items() for src in (0, 1)]
    cases.append((car_lib.car_helpers.FRAME_FINGERPRINT, car_model, can))

    # case 2 - no matches, make sure we keep going for 100 frames
    can = [car_lib.CanData(address=1, dat=b'\x00' * 1, src=src) for src in (0, 1)]  # uncommon address
    cases.append((car_lib.car_helpers.FRAME_FINGERPRINT, None, can))

    # case 3 - multiple matches, make sure we keep going for 200 frames to try to eliminate some
    can = [car_lib.CanData(address=2016, dat=b'\x00' * 8, src=src) for src in (0, 1)]  # common address
    cases.append((car_lib.car_helpers.FRAME_FINGERPRINT * 2, None, can))

    for expected_frames, car_model, can in cases:
      with subtests.test(expected_frames=expected_frames, car_model=car_model):
        frames = 0

        def can_recv(**kwargs):
          nonlocal frames
          frames += 1
          return [can]  # noqa: B023

        car_fingerprint, _ = car_lib.car_helpers.can_fingerprint(can_recv)
        assert car_fingerprint == car_model
        assert frames == expected_frames + 2  # TODO: fix extra frames
