import unittest
from parameterized import parameterized

from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import FRAME_FINGERPRINT, can_fingerprint
from opendbc.car.fingerprints import _FINGERPRINTS as FINGERPRINTS


class TestCanFingerprint(unittest.TestCase):

    @parameterized.expand([
        (car_model, fingerprint)
        for car_model, fps in FINGERPRINTS.items()
        for fingerprint in fps
    ])
    def test_can_fingerprint(self, car_model, fingerprint):
        """Tests online fingerprinting function on offline fingerprints"""
        can = [
            CanData(address=addr, dat=b'\x00' * length, src=src)
            for addr, length in fingerprint.items()
            for src in (0, 1)
        ]
        fingerprint_iter = iter([can])
        car_fp, finger = can_fingerprint(
            lambda **kwargs: [next(fingerprint_iter, [])]  # noqa: B023
        )
        self.assertEqual(car_fp, car_model)
        self.assertEqual(finger[0], fingerprint)
        self.assertEqual(finger[1], fingerprint)
        self.assertEqual(finger[2], {})

    def test_timing(self):
        """Ensure we iterate the correct number of frames before deciding"""
        # pick a known model fingerprint
        model = "CHEVROLET_BOLT_EUV"
        fp = FINGERPRINTS[model][0]

        cases = [
            # (frames_to_run, expected_car, can_frame)
            (FRAME_FINGERPRINT, model, [
                CanData(address=addr, dat=b'\x00' * length, src=src)
                for addr, length in fp.items()
                for src in (0, 1)
            ]),
            (FRAME_FINGERPRINT, None, [
                CanData(address=1, dat=b'\x00', src=src)
                for src in (0, 1)
            ]),
            (FRAME_FINGERPRINT * 2, None, [
                CanData(address=2016, dat=b'\x00' * 8, src=src)
                for src in (0, 1)
            ]),
        ]

        for expected_frames, expected_car, can in cases:
            with self.subTest(expected_frames=expected_frames, car_model=expected_car):
                frames = 0

                def can_recv(**kwargs):
                    nonlocal frames
                    frames += 1
                    return [can]

                car_fp, _ = can_fingerprint(can_recv)
                self.assertEqual(car_fp, expected_car)
                # implementation currently does +2 extra frames
                self.assertEqual(frames, expected_frames + 2)


if __name__ == "__main__":
    unittest.main()
