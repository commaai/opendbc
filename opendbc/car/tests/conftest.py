from opendbc.car.car_helpers import _get_interface_names, load_interfaces
import pytest


# imports from directory opendbc/car/<name>/
@pytest.fixture(scope="module")
def interfaces():
    interface_names = _get_interface_names()
    return load_interfaces(interface_names)
