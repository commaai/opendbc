from typing import get_args

from opendbc.car.body.interface import BodyInterface
from opendbc.car.chrysler.interface import ChryslerInterface
from opendbc.car.ford.interface import FordInterface
from opendbc.car.gm.interface import GMInterface
from opendbc.car.honda.interface import HondaInterface
from opendbc.car.hyundai.interface import HyundaiInterface
from opendbc.car.mazda.interface import MazdaInterface
from opendbc.car.mock.interface import MockInterface
from opendbc.car.nissan.interface import NissanInterface
from opendbc.car.psa.interface import PSAInterface
from opendbc.car.rivian.interface import RivianInterface
from opendbc.car.subaru.interface import SubaruInterface
from opendbc.car.tesla.interface import TeslaInterface
from opendbc.car.toyota.interface import ToyotaInterface
from opendbc.car.volkswagen.interface import VolkswagenInterface

# Every brand's Interface, carrying its platform enum, firmware versions, and
# query config as class attributes (see CarInterfaceBase). This is the single
# import point: given an Interface you have typed access to everything about
# that brand without magic module-level names.
ALL_INTERFACES: tuple = (BodyInterface, ChryslerInterface, FordInterface, GMInterface, HondaInterface,
                        HyundaiInterface, MazdaInterface, MockInterface, NissanInterface, PSAInterface,
                        RivianInterface, SubaruInterface, TeslaInterface, ToyotaInterface, VolkswagenInterface)

# Platform enums (lightweight type union for annotations)
Platform = (BodyInterface.CAR | ChryslerInterface.CAR | FordInterface.CAR | GMInterface.CAR |
            HondaInterface.CAR | HyundaiInterface.CAR | MazdaInterface.CAR | MockInterface.CAR |
            NissanInterface.CAR | PSAInterface.CAR | RivianInterface.CAR | SubaruInterface.CAR |
            TeslaInterface.CAR | ToyotaInterface.CAR | VolkswagenInterface.CAR)
BRANDS = get_args(Platform)

PLATFORMS: dict[str, Platform] = {str(platform): platform for brand in BRANDS for platform in brand}

# Aggregated firmware/fingerprint data, built from the Interface classes.
FW_VERSIONS: dict = {p: v for _ci in ALL_INTERFACES for p, v in getattr(_ci, 'FW_VERSIONS', {}).items()}
FINGERPRINTS: dict = {p: v for _ci in ALL_INTERFACES for p, v in getattr(_ci, 'FINGERPRINTS', {}).items()}

# Per-brand data, keyed by brand name string.
VERSIONS: dict = {_ci.BRAND: _ci.FW_VERSIONS for _ci in ALL_INTERFACES if hasattr(_ci, 'FW_VERSIONS')}
FW_QUERY_CONFIGS: dict = {_ci.BRAND: _ci.FW_QUERY_CONFIG for _ci in ALL_INTERFACES if hasattr(_ci, 'FW_QUERY_CONFIG')}
FOOTNOTES: list = [_ci.Footnote for _ci in ALL_INTERFACES if hasattr(_ci, 'Footnote')]
