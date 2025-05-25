import pytest
import sys

# Exit code 0 for success, 1 for test failures, >1 for pytest errors
exit_code = pytest.main(["opendbc/can/tests/test_packer_parser.py"])
sys.exit(exit_code)
