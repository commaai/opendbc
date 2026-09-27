import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from opendbc.car.logreader import capnp_log
from opendbc.car.tests import test_models


class TestModelDataCache(unittest.TestCase):
  def test_fixture_roundtrip(self):
    model = type("TestFixture", (test_models.TestCarModelBase,), {
      "platform": None, "test_route": test_models.CarTestRoute("test/route", None, 0),
    })
    with tempfile.TemporaryDirectory() as tmp:
      path = Path(tmp) / "rlog"
      with path.open("wb") as output:
        params = capnp_log.Event.new_message(logMonoTime=0)
        params.init("carParams")
        params.carParams.carFingerprint = "MOCK"
        params.carParams.openpilotLongitudinalControl = True
        params.carParams.carFw = [{"ecu": "eps", "fwVersion": b"firmware", "address": 0x123},
                                  {"ecu": 65535, "fwVersion": b"unknown enum", "address": 0x456}]
        output.write(params.to_bytes())
        for i in range(5001):
          event = capnp_log.Event.new_message(logMonoTime=i * 2 + 1)
          event.can = [{"address": 0x123, "dat": b"12345678", "src": 0}]
          output.write(event.to_bytes())
          if i == 1:
            panda = capnp_log.Event.new_message(logMonoTime=i * 2 + 2)
            panda.pandaStates = [{"safetyModel": "toyota"}]
            output.write(panda.to_bytes())

      with patch.object(test_models, "MODEL_DATA_CACHE_ROOT", Path(tmp) / "cache"):
        with patch.object(test_models, "get_cached_segment", return_value=path):
          firmware, messages, alpha_long = model.get_testing_data()
        expected = ([fw.as_builder().to_bytes_packed() for fw in firmware], messages, alpha_long,
                    model.fingerprint, model.elm_frame, model.car_safety_mode_frame, model.platform)
        self.assertEqual(expected[4:], (2, 2, "MOCK"))
        model.platform = None
        model.fingerprint = {}
        model.elm_frame = model.car_safety_mode_frame = None
        with patch.object(test_models, "get_cached_segment", side_effect=AssertionError("cache miss")):
          firmware, messages, alpha_long = model.get_testing_data()
        self.assertEqual(([fw.as_builder().to_bytes_packed() for fw in firmware], messages, alpha_long,
                          model.fingerprint, model.elm_frame, model.car_safety_mode_frame, model.platform), expected)
