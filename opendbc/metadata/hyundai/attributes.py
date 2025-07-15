from dataclasses import dataclass, field
from enum import Enum
from opendbc.car.docs_definitions import CarDocs, CarFootnote, CarHarness, CarParts, Column, Device
from opendbc.car.common.conversions import Conversions as CV


class Footnote(Enum):
  CANFD = CarFootnote(
    "Requires a <a href=\"https://comma.ai/shop/can-fd-panda-kit\" target=\"_blank\">CAN FD panda kit</a> if not using " +
    "comma 3X for this <a href=\"https://en.wikipedia.org/wiki/CAN_FD\" target=\"_blank\">CAN FD car</a>.",
    Column.MODEL)


@dataclass
class HyundaiCarDocs(CarDocs):
  package: str = "Smart Cruise Control (SCC)"


METADATA = {
  # Hyundai
  "HYUNDAI_AZERA_6TH_GEN": [
    HyundaiCarDocs(
      name="Hyundai Azera 2022",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_k]),
    ),
  ],
  "HYUNDAI_AZERA_HEV_6TH_GEN": [
    HyundaiCarDocs(
      name="Hyundai Azera Hybrid 2019",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_c]),
    ),
    HyundaiCarDocs(
      name="Hyundai Azera Hybrid 2020",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_k]),
    ),
  ],
  "HYUNDAI_ELANTRA": [
    # TODO: 2017-18 could be Hyundai G
    HyundaiCarDocs(
      name="Hyundai Elantra 2017-18",
      min_enable_speed=19 * CV.MPH_TO_MS,
      car_parts=CarParts.common([CarHarness.hyundai_b]),
    ),
    HyundaiCarDocs(
      name="Hyundai Elantra 2019",
      min_enable_speed=19 * CV.MPH_TO_MS,
      car_parts=CarParts.common([CarHarness.hyundai_g]),
    ),
  ],
  "HYUNDAI_ELANTRA_GT_I30": [
    HyundaiCarDocs(
      name="Hyundai Elantra GT 2017-20",
      car_parts=CarParts.common([CarHarness.hyundai_e]),
    ),
    HyundaiCarDocs(
      name="Hyundai i30 2017-19",
      car_parts=CarParts.common([CarHarness.hyundai_e]),
    ),
  ],
  "HYUNDAI_ELANTRA_2021": [
    HyundaiCarDocs(
      name="Hyundai Elantra 2021-23",
      video="https://youtu.be/_EdYQtV52-c",
      car_parts=CarParts.common([CarHarness.hyundai_k]),
    ),
  ],
  "HYUNDAI_ELANTRA_HEV_2021": [
    HyundaiCarDocs(
      name="Hyundai Elantra Hybrid 2021-23",
      video="https://youtu.be/_EdYQtV52-c",
      car_parts=CarParts.common([CarHarness.hyundai_k]),
    ),
  ],
  "HYUNDAI_GENESIS": [
    # TODO: check 2015 packages
    HyundaiCarDocs(
      name="Hyundai Genesis 2015-16",
      min_enable_speed=19 * CV.MPH_TO_MS,
      car_parts=CarParts.common([CarHarness.hyundai_j]),
    ),
    HyundaiCarDocs(
      name="Genesis G80 2017",
      package="All",
      min_enable_speed=19 * CV.MPH_TO_MS,
      car_parts=CarParts.common([CarHarness.hyundai_j]),
    ),
  ],
  "HYUNDAI_IONIQ": [
    HyundaiCarDocs(
      name="Hyundai Ioniq Hybrid 2017-19",
      car_parts=CarParts.common([CarHarness.hyundai_c]),
    ),
  ],
  "HYUNDAI_IONIQ_HEV_2022": [
    HyundaiCarDocs(
      name="Hyundai Ioniq Hybrid 2020-22",
      car_parts=CarParts.common([CarHarness.hyundai_h]),
    ),
  ],
  "HYUNDAI_IONIQ_EV_LTD": [
    HyundaiCarDocs(
      name="Hyundai Ioniq Electric 2019",
      car_parts=CarParts.common([CarHarness.hyundai_c]),
    ),
  ],
  "HYUNDAI_IONIQ_EV_2020": [
    HyundaiCarDocs(
      name="Hyundai Ioniq Electric 2020",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_h]),
    ),
  ],
  "HYUNDAI_IONIQ_PHEV_2019": [
    HyundaiCarDocs(
      name="Hyundai Ioniq Plug-in Hybrid 2019",
      car_parts=CarParts.common([CarHarness.hyundai_c]),
    ),
  ],
  "HYUNDAI_IONIQ_PHEV": [
    HyundaiCarDocs(
      name="Hyundai Ioniq Plug-in Hybrid 2020-22",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_h]),
    ),
  ],
  "HYUNDAI_KONA": [
    HyundaiCarDocs(
      name="Hyundai Kona 2020",
      min_enable_speed=6 * CV.MPH_TO_MS,
      car_parts=CarParts.common([CarHarness.hyundai_b]),
    ),
  ],
  "HYUNDAI_KONA_2022": [
    HyundaiCarDocs(
      name="Hyundai Kona 2022",
      car_parts=CarParts.common([CarHarness.hyundai_o]),
    ),
  ],
  "HYUNDAI_KONA_EV": [
    HyundaiCarDocs(
      name="Hyundai Kona Electric 2018-21",
      car_parts=CarParts.common([CarHarness.hyundai_g]),
    ),
  ],
  "HYUNDAI_KONA_EV_2022": [
    HyundaiCarDocs(
      name="Hyundai Kona Electric 2022-23",
      car_parts=CarParts.common([CarHarness.hyundai_o]),
    ),
  ],
  "HYUNDAI_KONA_EV_2ND_GEN": [
    HyundaiCarDocs(
      name="Hyundai Kona Electric (with HDA II, Korea only) 2023",
      video="https://www.youtube.com/watch?v=U2fOCmcQ8hw",
      car_parts=CarParts.common([CarHarness.hyundai_r]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "HYUNDAI_KONA_HEV": [
    # TODO: check packages
    HyundaiCarDocs(
      name="Hyundai Kona Hybrid 2020",
      car_parts=CarParts.common([CarHarness.hyundai_i]),
    ),
  ],
  "HYUNDAI_NEXO_1ST_GEN": [
    HyundaiCarDocs(
      name="Hyundai Nexo 2021",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_h]),
    ),
  ],
  "HYUNDAI_SANTA_FE": [
    HyundaiCarDocs(
      name="Hyundai Santa Fe 2019-20",
      package="All",
      video="https://youtu.be/bjDR0YjM__s",
      car_parts=CarParts.common([CarHarness.hyundai_d]),
    ),
  ],
  "HYUNDAI_SANTA_FE_2022": [
    HyundaiCarDocs(
      name="Hyundai Santa Fe 2021-23",
      package="All",
      video="https://youtu.be/VnHzSTygTS4",
      car_parts=CarParts.common([CarHarness.hyundai_l]),
    ),
  ],
  "HYUNDAI_SANTA_FE_HEV_2022": [
    HyundaiCarDocs(
      name="Hyundai Santa Fe Hybrid 2022-23",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_l]),
    ),
  ],
  "HYUNDAI_SANTA_FE_PHEV_2022": [
    HyundaiCarDocs(
      name="Hyundai Santa Fe Plug-in Hybrid 2022-23",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_l]),
    ),
  ],
  "HYUNDAI_SONATA": [
    HyundaiCarDocs(
      name="Hyundai Sonata 2020-23",
      package="All",
      video="https://www.youtube.com/watch?v=ix63r9kE3Fw",
      car_parts=CarParts.common([CarHarness.hyundai_a]),
    ),
  ],
  "HYUNDAI_SONATA_LF": [
    HyundaiCarDocs(
      name="Hyundai Sonata 2018-19",
      car_parts=CarParts.common([CarHarness.hyundai_e]),
    ),
  ],
  "HYUNDAI_STARIA_4TH_GEN": [
    HyundaiCarDocs(
      name="Hyundai Staria 2023",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_k]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "HYUNDAI_TUCSON": [
    HyundaiCarDocs(
      name="Hyundai Tucson 2021",
      min_enable_speed=19 * CV.MPH_TO_MS,
      car_parts=CarParts.common([CarHarness.hyundai_l]),
    ),
    HyundaiCarDocs(
      name="Hyundai Tucson Diesel 2019",
      car_parts=CarParts.common([CarHarness.hyundai_l]),
    ),
  ],
  "HYUNDAI_PALISADE": [
    HyundaiCarDocs(
      name="Hyundai Palisade 2020-22",
      package="All",
      video="https://youtu.be/TAnDqjF4fDY?t=456",
      car_parts=CarParts.common([CarHarness.hyundai_h]),
    ),
    HyundaiCarDocs(
      name="Kia Telluride 2020-22",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_h]),
    ),
  ],
  "HYUNDAI_VELOSTER": [
    HyundaiCarDocs(
      name="Hyundai Veloster 2019-20",
      min_enable_speed=5. * CV.MPH_TO_MS,
      car_parts=CarParts.common([CarHarness.hyundai_e]),
    ),
  ],
  "HYUNDAI_SONATA_HYBRID": [
    HyundaiCarDocs(
      name="Hyundai Sonata Hybrid 2020-23",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_a]),
    ),
  ],
  "HYUNDAI_IONIQ_5": [
    HyundaiCarDocs(
      name="Hyundai Ioniq 5 (Southeast Asia and Europe only) 2022-24",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_q]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Hyundai Ioniq 5 (without HDA II) 2022-24",
      package="Highway Driving Assist",
      car_parts=CarParts.common([CarHarness.hyundai_k]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Hyundai Ioniq 5 (with HDA II) 2022-24",
      package="Highway Driving Assist II",
      car_parts=CarParts.common([CarHarness.hyundai_q]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "HYUNDAI_IONIQ_6": [
    HyundaiCarDocs(
      name="Hyundai Ioniq 6 (with HDA II) 2023-24",
      package="Highway Driving Assist II",
      car_parts=CarParts.common([CarHarness.hyundai_p]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "HYUNDAI_TUCSON_4TH_GEN": [
    HyundaiCarDocs(
      name="Hyundai Tucson 2022",
      car_parts=CarParts.common([CarHarness.hyundai_n]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Hyundai Tucson 2023-24",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_n]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Hyundai Tucson Hybrid 2022-24",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_n]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Hyundai Tucson Plug-in Hybrid 2024",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_n]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "HYUNDAI_SANTA_CRUZ_1ST_GEN": [
    HyundaiCarDocs(
      name="Hyundai Santa Cruz 2022-24",
      car_parts=CarParts.common([CarHarness.hyundai_n]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "HYUNDAI_CUSTIN_1ST_GEN": [
    HyundaiCarDocs(
      name="Hyundai Custin 2023",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_k]),
    ),
  ],

  # Kia
  "KIA_FORTE": [
    HyundaiCarDocs(
      name="Kia Forte 2019-21",
      min_enable_speed=6 * CV.MPH_TO_MS,
      car_parts=CarParts.common([CarHarness.hyundai_g]),
    ),
    HyundaiCarDocs(
      name="Kia Forte 2022-23",
      car_parts=CarParts.common([CarHarness.hyundai_e]),
    ),
  ],
  "KIA_K5_2021": [
    HyundaiCarDocs(
      name="Kia K5 2021-24",
      car_parts=CarParts.common([CarHarness.hyundai_a]),
    ),
  ],
  "KIA_K5_HEV_2020": [
    HyundaiCarDocs(
      name="Kia K5 Hybrid 2020-22",
      car_parts=CarParts.common([CarHarness.hyundai_a]),
    ),
  ],
  "KIA_K8_HEV_1ST_GEN": [
    HyundaiCarDocs(
      name="Kia K8 Hybrid (with HDA II) 2023",
      package="Highway Driving Assist II",
      car_parts=CarParts.common([CarHarness.hyundai_q]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "KIA_NIRO_EV": [
    HyundaiCarDocs(
      name="Kia Niro EV 2019",
      package="All",
      video="https://www.youtube.com/watch?v=lT7zcG6ZpGo",
      car_parts=CarParts.common([CarHarness.hyundai_h]),
    ),
    HyundaiCarDocs(
      name="Kia Niro EV 2020",
      package="All",
      video="https://www.youtube.com/watch?v=lT7zcG6ZpGo",
      car_parts=CarParts.common([CarHarness.hyundai_f]),
    ),
    HyundaiCarDocs(
      name="Kia Niro EV 2021",
      package="All",
      video="https://www.youtube.com/watch?v=lT7zcG6ZpGo",
      car_parts=CarParts.common([CarHarness.hyundai_c]),
    ),
    HyundaiCarDocs(
      name="Kia Niro EV 2022",
      package="All",
      video="https://www.youtube.com/watch?v=lT7zcG6ZpGo",
      car_parts=CarParts.common([CarHarness.hyundai_h]),
    ),
  ],
  "KIA_NIRO_EV_2ND_GEN": [
    HyundaiCarDocs(
      name="Kia Niro EV (without HDA II) 2023-25",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_a]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Kia Niro EV (with HDA II) 2025",
      package="Highway Driving Assist II",
      car_parts=CarParts.common([CarHarness.hyundai_r]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "KIA_NIRO_PHEV": [
    HyundaiCarDocs(
      name="Kia Niro Hybrid 2018",
      min_enable_speed=10. * CV.MPH_TO_MS,
      car_parts=CarParts.common([CarHarness.hyundai_c]),
    ),
    HyundaiCarDocs(
      name="Kia Niro Plug-in Hybrid 2018-19",
      package="All",
      min_enable_speed=10. * CV.MPH_TO_MS,
      car_parts=CarParts.common([CarHarness.hyundai_c]),
    ),
    HyundaiCarDocs(
      name="Kia Niro Plug-in Hybrid 2020",
      car_parts=CarParts.common([CarHarness.hyundai_d]),
    ),
  ],
  "KIA_NIRO_PHEV_2022": [
    HyundaiCarDocs(
      name="Kia Niro Plug-in Hybrid 2021",
      car_parts=CarParts.common([CarHarness.hyundai_d]),
    ),
    HyundaiCarDocs(
      name="Kia Niro Plug-in Hybrid 2022",
      car_parts=CarParts.common([CarHarness.hyundai_f]),
    ),
  ],
  "KIA_NIRO_HEV_2021": [
    HyundaiCarDocs(
      name="Kia Niro Hybrid 2021",
      car_parts=CarParts.common([CarHarness.hyundai_d]),
    ),
    HyundaiCarDocs(
      name="Kia Niro Hybrid 2022",
      car_parts=CarParts.common([CarHarness.hyundai_f]),
    ),
  ],
  "KIA_NIRO_HEV_2ND_GEN": [
    HyundaiCarDocs(
      name="Kia Niro Hybrid 2023",
      car_parts=CarParts.common([CarHarness.hyundai_a]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "KIA_OPTIMA_G4": [
    # TODO: may support 2016, 2018
    HyundaiCarDocs(
      name="Kia Optima 2017",
      package="Advanced Smart Cruise Control",
      car_parts=CarParts.common([CarHarness.hyundai_b]),
    ),
  ],
  "KIA_OPTIMA_G4_FL": [
    HyundaiCarDocs(
      name="Kia Optima 2019-20",
      car_parts=CarParts.common([CarHarness.hyundai_g]),
    ),
  ],
  "KIA_OPTIMA_H": [
    HyundaiCarDocs(
      name="Kia Optima Hybrid 2017",
      package="Advanced Smart Cruise Control",
      car_parts=CarParts.common([CarHarness.hyundai_c]),
    ),
  ],
  "KIA_OPTIMA_H_G4_FL": [
    HyundaiCarDocs(
      name="Kia Optima Hybrid 2019",
      car_parts=CarParts.common([CarHarness.hyundai_h]),
    ),
  ],
  "KIA_SELTOS": [
    HyundaiCarDocs(
      name="Kia Seltos 2021",
      car_parts=CarParts.common([CarHarness.hyundai_a]),
    ),
  ],
  "KIA_SPORTAGE_5TH_GEN": [
    HyundaiCarDocs(
      name="Kia Sportage 2023-24",
      car_parts=CarParts.common([CarHarness.hyundai_n]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Kia Sportage Hybrid 2023",
      car_parts=CarParts.common([CarHarness.hyundai_n]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "KIA_SORENTO": [
    HyundaiCarDocs(
      name="Kia Sorento 2018",
      package="Advanced Smart Cruise Control & LKAS",
      video="https://www.youtube.com/watch?v=Fkh3s6WHJz8",
      car_parts=CarParts.common([CarHarness.hyundai_e]),
    ),
    HyundaiCarDocs(
      name="Kia Sorento 2019",
      video="https://www.youtube.com/watch?v=Fkh3s6WHJz8",
      car_parts=CarParts.common([CarHarness.hyundai_e]),
    ),
  ],
  "KIA_SORENTO_4TH_GEN": [
    HyundaiCarDocs(
      name="Kia Sorento 2021-23",
      car_parts=CarParts.common([CarHarness.hyundai_k]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "KIA_SORENTO_HEV_4TH_GEN": [
    HyundaiCarDocs(
      name="Kia Sorento Hybrid 2021-23",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_a]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Kia Sorento Plug-in Hybrid 2022-23",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_a]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "KIA_STINGER": [
    HyundaiCarDocs(
      name="Kia Stinger 2018-20",
      video="https://www.youtube.com/watch?v=MJ94qoofYw0",
      car_parts=CarParts.common([CarHarness.hyundai_c]),
    ),
  ],
  "KIA_STINGER_2022": [
    HyundaiCarDocs(
      name="Kia Stinger 2022-23",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_k]),
    ),
  ],
  "KIA_CEED": [
    HyundaiCarDocs(
      name="Kia Ceed 2019-21",
      car_parts=CarParts.common([CarHarness.hyundai_e]),
    ),
  ],
  "KIA_EV6": [
    HyundaiCarDocs(
      name="Kia EV6 (Southeast Asia only) 2022-24",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_p]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Kia EV6 (without HDA II) 2022-24",
      package="Highway Driving Assist",
      car_parts=CarParts.common([CarHarness.hyundai_l]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Kia EV6 (with HDA II) 2022-24",
      package="Highway Driving Assist II",
      car_parts=CarParts.common([CarHarness.hyundai_p]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "KIA_CARNIVAL_4TH_GEN": [
    HyundaiCarDocs(
      name="Kia Carnival 2022-24",
      car_parts=CarParts.common([CarHarness.hyundai_a]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Kia Carnival (China only) 2023",
      car_parts=CarParts.common([CarHarness.hyundai_k]),
      footnotes=[Footnote.CANFD],
    ),
  ],

  # Genesis
  "GENESIS_GV60_EV_1ST_GEN": [
    HyundaiCarDocs(
      name="Genesis GV60 (Advanced Trim) 2023",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_a]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Genesis GV60 (Performance Trim) 2022-23",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_k]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "GENESIS_G70": [
    HyundaiCarDocs(
      name="Genesis G70 2018",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_f]),
    ),
  ],
  "GENESIS_G70_2020": [
    # TODO: 2021 MY harness is unknown
    HyundaiCarDocs(
      name="Genesis G70 2019-21",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_f]),
    ),
    # TODO: From 3.3T Sport Advanced 2022 & Prestige 2023 Trim, 2.0T is unknown
    HyundaiCarDocs(
      name="Genesis G70 2022-23",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_l]),
    ),
  ],
  "GENESIS_GV70_1ST_GEN": [
    # TODO: Hyundai P is likely the correct harness for HDA II for 2.5T (unsupported due to missing ADAS ECU, is that the radar?)
    HyundaiCarDocs(
      name="Genesis GV70 (2.5T Trim, without HDA II) 2022-24",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_l]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Genesis GV70 (3.5T Trim, without HDA II) 2022-23",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_m]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "GENESIS_GV70_ELECTRIFIED_1ST_GEN": [
    HyundaiCarDocs(
      name="Genesis GV70 Electrified (Australia Only) 2022",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_q]),
      footnotes=[Footnote.CANFD],
    ),
    HyundaiCarDocs(
      name="Genesis GV70 Electrified (with HDA II) 2023-24",
      package="Highway Driving Assist II",
      car_parts=CarParts.common([CarHarness.hyundai_q]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "GENESIS_G80": [
    HyundaiCarDocs(
      name="Genesis G80 2018-19",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_h]),
    ),
  ],
  "GENESIS_G80_2ND_GEN_FL": [
    HyundaiCarDocs(
      name="Genesis G80 (2.5T Advanced Trim, with HDA II) 2024",
      package="Highway Driving Assist II",
      car_parts=CarParts.common([CarHarness.hyundai_p]),
      footnotes=[Footnote.CANFD],
    ),
  ],
  "GENESIS_G90": [
    HyundaiCarDocs(
      name="Genesis G90 2017-20",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_c]),
    ),
  ],
  "GENESIS_GV80": [
    HyundaiCarDocs(
      name="Genesis GV80 2023",
      package="All",
      car_parts=CarParts.common([CarHarness.hyundai_m]),
      footnotes=[Footnote.CANFD],
    ),
  ],
} 