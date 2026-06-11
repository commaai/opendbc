#!/usr/bin/env python3
import unittest

from opendbc.safety.tests.libsafety.libsafety_py import _build_libsafety


class TestReleaseBuild(unittest.TestCase):
  def test_build_without_allow_debug(self):
    # panda's release firmware builds the safety code without ALLOW_DEBUG
    _build_libsafety(release=True)
    assert False


if __name__ == "__main__":
  unittest.main()
