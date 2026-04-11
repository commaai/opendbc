"""
Tests for Tesla fuzzy fingerprinting.
Place in: opendbc/car/tesla/tests/test_tesla_fuzzy_fingerprint.py

These tests verify that Tesla's fuzzy fingerprinting correctly identifies
car models based on the model identifier code in EPS firmware strings,
even for firmware versions not in the database.
"""
import unittest

from opendbc.car.tesla.values import (
    CAR,
    TESLA_FW_PATTERN,
    get_platform_codes,
    match_fw_to_car_fuzzy,
)
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu
EPS_ADDR = (0x730, None)


class TestTeslaFwPattern(unittest.TestCase):
    """Test that the firmware regex pattern matches all known Tesla EPS firmware versions."""

    def test_pattern_matches_all_known_fw(self):
        for car_model, ecus in FW_VERSIONS.items():
            for ecu, versions in ecus.items():
                for fw in versions:
                    with self.subTest(car_model=car_model, fw=fw):
                        match = TESLA_FW_PATTERN.match(fw)
                        self.assertIsNotNone(match, f"Pattern failed to match: {fw}")


class TestGetPlatformCodes(unittest.TestCase):
    """Test platform code extraction from firmware version strings."""

    def test_model_3_codes_are_E(self):
        codes = get_platform_codes(FW_VERSIONS[CAR.TESLA_MODEL_3][(Ecu.eps, 0x730, None)])
        model_letters = {code[0] for code in codes}
        self.assertEqual(model_letters, {b'E'})

    def test_model_y_codes_are_Y(self):
        codes = get_platform_codes(FW_VERSIONS[CAR.TESLA_MODEL_Y][(Ecu.eps, 0x730, None)])
        model_letters = {code[0] for code in codes}
        self.assertEqual(model_letters, {b'Y'})

    def test_model_x_codes_are_X(self):
        codes = get_platform_codes(FW_VERSIONS[CAR.TESLA_MODEL_X][(Ecu.eps, 0x730, None)])
        model_letters = {code[0] for code in codes}
        self.assertEqual(model_letters, {b'X'})

    def test_no_model_letter_overlap(self):
        m3 = {c[0] for c in get_platform_codes(FW_VERSIONS[CAR.TESLA_MODEL_3][(Ecu.eps, 0x730, None)])}
        my = {c[0] for c in get_platform_codes(FW_VERSIONS[CAR.TESLA_MODEL_Y][(Ecu.eps, 0x730, None)])}
        mx = {c[0] for c in get_platform_codes(FW_VERSIONS[CAR.TESLA_MODEL_X][(Ecu.eps, 0x730, None)])}
        self.assertFalse(m3 & my, "Model 3 and Model Y share model letters")
        self.assertFalse(m3 & mx, "Model 3 and Model X share model letters")
        self.assertFalse(my & mx, "Model Y and Model X share model letters")

    def test_empty_input(self):
        self.assertEqual(get_platform_codes([]), set())
        self.assertEqual(get_platform_codes(set()), set())

    def test_garbage_input(self):
        self.assertEqual(get_platform_codes([b'garbage']), set())
        self.assertEqual(get_platform_codes([b'']), set())


class TestTeslaFuzzyMatch(unittest.TestCase):
    """Test the fuzzy fingerprinting function for Tesla vehicles."""

    @property
    def offline_fw(self):
        """Build offline FW dict in the format match_fw_to_car_fuzzy expects."""
        return {car.value: ecus for car, ecus in FW_VERSIONS.items()}

    def test_known_fw_matches_correct_model(self):
        """Every known FW version should fuzzy-match to its own model."""
        for car_model, ecus in FW_VERSIONS.items():
            for ecu, versions in ecus.items():
                for fw in versions:
                    with self.subTest(car_model=car_model, fw=fw):
                        live = {EPS_ADDR: {fw}}
                        result = match_fw_to_car_fuzzy(live, "", self.offline_fw)
                        self.assertIn(car_model.value, result)

    def test_known_fw_does_not_cross_match(self):
        """Model 3 FW should not match Model Y or X, etc."""
        model_map = {
            CAR.TESLA_MODEL_3: [CAR.TESLA_MODEL_Y, CAR.TESLA_MODEL_X],
            CAR.TESLA_MODEL_Y: [CAR.TESLA_MODEL_3, CAR.TESLA_MODEL_X],
            CAR.TESLA_MODEL_X: [CAR.TESLA_MODEL_3, CAR.TESLA_MODEL_Y],
        }
        for car_model, ecus in FW_VERSIONS.items():
            for ecu, versions in ecus.items():
                for fw in versions:
                    live = {EPS_ADDR: {fw}}
                    result = match_fw_to_car_fuzzy(live, "", self.offline_fw)
                    for wrong_model in model_map[car_model]:
                        with self.subTest(car_model=car_model, wrong=wrong_model, fw=fw):
                            self.assertNotIn(wrong_model.value, result)

    def test_unknown_model_3_fw(self):
        """Simulated future Model 3 firmware should still match."""
        unknown_fws = [
            b'TeMYG4_Main_0.0.0 (99),E4H016.01.0',
            b'TeMYG4_NewBuild_0.0.0 (1),E5017.01.0',
            b'TeM3_E014p12_0.0.0 (30),E014.20.00',
        ]
        for fw in unknown_fws:
            with self.subTest(fw=fw):
                live = {EPS_ADDR: {fw}}
                result = match_fw_to_car_fuzzy(live, "", self.offline_fw)
                self.assertIn(CAR.TESLA_MODEL_3.value, result)
                self.assertNotIn(CAR.TESLA_MODEL_Y.value, result)

    def test_unknown_model_y_fw(self):
        """Simulated future Model Y firmware should still match."""
        unknown_fws = [
            b'TeMYG4_Main_0.0.0 (99),Y4004.01.0',
            b'TeMYG4_Legacy3Y_0.0.0 (10),Y4003.10.0',
            b'TeM3_E014p10_0.0.0 (20),YP003.01.00',
        ]
        for fw in unknown_fws:
            with self.subTest(fw=fw):
                live = {EPS_ADDR: {fw}}
                result = match_fw_to_car_fuzzy(live, "", self.offline_fw)
                self.assertIn(CAR.TESLA_MODEL_Y.value, result)
                self.assertNotIn(CAR.TESLA_MODEL_3.value, result)

    def test_unknown_model_x_fw(self):
        """Simulated future Model X firmware should still match."""
        unknown_fws = [
            b'TeM3_SP_XP002p3_0.0.0 (50),XPR004.1.0',
            b'TeM3_SP_XP003p1_0.0.0 (1),XPR005.01.0',
        ]
        for fw in unknown_fws:
            with self.subTest(fw=fw):
                live = {EPS_ADDR: {fw}}
                result = match_fw_to_car_fuzzy(live, "", self.offline_fw)
                self.assertIn(CAR.TESLA_MODEL_X.value, result)
                self.assertNotIn(CAR.TESLA_MODEL_3.value, result)

    def test_malformed_fw_no_match(self):
        """Garbage firmware data should not produce any matches."""
        garbage = [b'garbage_data', b'', b'NotATesla_FW_1.0.0 (1),Z999.01.0', b'random bytes']
        for fw in garbage:
            with self.subTest(fw=fw):
                live = {EPS_ADDR: {fw}}
                result = match_fw_to_car_fuzzy(live, "", self.offline_fw)
                self.assertEqual(len(result), 0)

    def test_empty_live_fw(self):
        result = match_fw_to_car_fuzzy({}, "", self.offline_fw)
        self.assertEqual(len(result), 0)

    def test_wrong_ecu_address(self):
        live = {(0x999, None): {b'TeMYG4_Main_0.0.0 (77),E4H015.04.5'}}
        result = match_fw_to_car_fuzzy(live, "", self.offline_fw)
        self.assertEqual(len(result), 0)

    def test_multiple_fw_at_same_address(self):
        """Multiple firmware responses at the same ECU address should work."""
        live = {EPS_ADDR: {
            b'TeMYG4_Main_0.0.0 (77),E4H015.04.5',
            b'TeMYG4_Main_0.0.0 (78),E4HP015.05.0',
        }}
        result = match_fw_to_car_fuzzy(live, "", self.offline_fw)
        self.assertIn(CAR.TESLA_MODEL_3.value, result)
        self.assertNotIn(CAR.TESLA_MODEL_Y.value, result)


if __name__ == '__main__':
    unittest.main()
