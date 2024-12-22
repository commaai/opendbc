from parameterized import parameterized

from opendbc.car.byd.fingerprints import FINGERPRINTS


class TestBYDFingerprint:
    @parameterized.expand(FINGERPRINTS.items())
    def test_can_fingerprints(self, car_model, fingerprints):
        assert len(fingerprints) > 0

        assert all(len(finger) for finger in fingerprints)
