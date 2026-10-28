import re
from collections import namedtuple
import copy
from dataclasses import dataclass, field
from enum import Enum

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams

GOOD_TORQUE_THRESHOLD = 1.0  # m/s^2
MODEL_YEARS_RE = r"(?<= )((\d{4}-\d{2})|(\d{4}))(,|$)"


class Column(Enum):
  MAKE = "Make"
  MODEL = "Model"
  PACKAGE = "Supported Package"
  LONGITUDINAL = "ACC"
  FSR_LONGITUDINAL = "No ACC accel below"
  FSR_STEERING = "No ALC below"
  STEERING_TORQUE = "Steering Torque"
  AUTO_RESUME = "Resume from stop"
  HARDWARE = "Hardware Needed"
  VIDEO = "Video"
  SETUP_VIDEO = "Setup Video"


class ExtraCarsColumn(Enum):
  MAKE = "Make"
  MODEL = "Model"
  PACKAGE = "Package"
  SUPPORT = "Support Level"


class SupportType(Enum):
  UPSTREAM = "Upstream"             # Actively maintained by comma, plug-and-play in release versions of openpilot
  REVIEW = "Under review"           # Dashcam, but planned for official support after safety validation
  DASHCAM = "Dashcam mode"          # Dashcam, but may be drivable in a community fork
  COMMUNITY = "Community"           # Not upstream, but available in a custom community fork, not validated by comma
  CUSTOM = "Custom"                 # Upstream, but don't have a harness available or need an unusual custom install
  INCOMPATIBLE = "Not compatible"   # Known fundamental incompatibility such as Flexray or hydraulic power steering


class Star(Enum):
  FULL = "full"
  HALF = "half"
  EMPTY = "empty"


# A part + its comprised parts
@dataclass
class BasePart:
  name: str
  parts: list[Enum] = field(default_factory=list)

  def all_parts(self):
    # Recursively get all parts
    _parts = 'parts'
    parts = []
    parts.extend(getattr(self, _parts))
    for part in getattr(self, _parts):
      parts.extend(part.value.all_parts())

    return parts


class EnumBase(Enum):
  @property
  def part_type(self):
    return PartType(self.__class__)


class Mount(EnumBase):
  mount = BasePart("mount")


class Cable(EnumBase):
  long_obdc_cable = BasePart("long OBD-C cable (9.5 ft)")
  usb_a_2_a_cable = BasePart("USB A-A cable")
  usbc_otg_cable = BasePart("USB C OTG cable")
  obd_c_cable_2ft = BasePart("OBD-C cable (2 ft)")


class Accessory(EnumBase):
  harness_box = BasePart("harness box")
  comma_power = BasePart("comma power v3")


class Tool(EnumBase):
  socket_8mm_deep = BasePart("Socket Wrench 8mm or 5/16\" (deep)")
  pry_tool = BasePart("Pry Tool")


@dataclass
class BaseCarHarness(BasePart):
  parts: list[Enum] = field(default_factory=lambda: [Accessory.harness_box, Accessory.comma_power])
  has_connector: bool = True  # without are hidden on the harness connector page


class CarHarness(EnumBase):
  nidec = BaseCarHarness("Honda Nidec connector")
  bosch_a = BaseCarHarness("Honda Bosch A connector")
  bosch_b = BaseCarHarness("Honda Bosch B connector")
  bosch_c = BaseCarHarness("Honda Bosch C connector")
  toyota_a = BaseCarHarness("Toyota A connector")
  toyota_b = BaseCarHarness("Toyota B connector")
  subaru_a = BaseCarHarness("Subaru A connector", parts=[Accessory.harness_box, Accessory.comma_power, Tool.socket_8mm_deep, Tool.pry_tool])
  subaru_b = BaseCarHarness("Subaru B connector", parts=[Accessory.harness_box, Accessory.comma_power, Tool.socket_8mm_deep, Tool.pry_tool])
  subaru_c = BaseCarHarness("Subaru C connector", parts=[Accessory.harness_box, Accessory.comma_power, Tool.socket_8mm_deep, Tool.pry_tool])
  subaru_d = BaseCarHarness("Subaru D connector", parts=[Accessory.harness_box, Accessory.comma_power, Tool.socket_8mm_deep, Tool.pry_tool])
  fca = BaseCarHarness("FCA connector")
  ram = BaseCarHarness("Ram connector")
  vw_a = BaseCarHarness("VW A connector")
  vw_j533 = BaseCarHarness("VW J533 connector", parts=[Accessory.harness_box, Cable.long_obdc_cable])
  vw_meb = BaseCarHarness("VW MEB connector", parts=[Accessory.harness_box, Cable.long_obdc_cable])
  hyundai_a = BaseCarHarness("Hyundai A connector")
  hyundai_b = BaseCarHarness("Hyundai B connector")
  hyundai_c = BaseCarHarness("Hyundai C connector")
  hyundai_d = BaseCarHarness("Hyundai D connector")
  hyundai_e = BaseCarHarness("Hyundai E connector")
  hyundai_f = BaseCarHarness("Hyundai F connector")
  hyundai_g = BaseCarHarness("Hyundai G connector")
  hyundai_h = BaseCarHarness("Hyundai H connector")
  hyundai_i = BaseCarHarness("Hyundai I connector")
  hyundai_j = BaseCarHarness("Hyundai J connector")
  hyundai_k = BaseCarHarness("Hyundai K connector")
  hyundai_l = BaseCarHarness("Hyundai L connector")
  hyundai_m = BaseCarHarness("Hyundai M connector")
  hyundai_n = BaseCarHarness("Hyundai N connector")
  hyundai_o = BaseCarHarness("Hyundai O connector")
  hyundai_p = BaseCarHarness("Hyundai P connector")
  hyundai_q = BaseCarHarness("Hyundai Q connector")
  hyundai_r = BaseCarHarness("Hyundai R connector")
  custom = BaseCarHarness("Developer connector")
  obd_ii = BaseCarHarness("OBD-II connector", parts=[Cable.long_obdc_cable], has_connector=False)
  gm = BaseCarHarness("GM connector", parts=[Accessory.harness_box])
  gmsdgm = BaseCarHarness("GM SDGM connector", parts=[Accessory.harness_box, Accessory.comma_power, Cable.long_obdc_cable])
  nissan_a = BaseCarHarness("Nissan A connector", parts=[Accessory.harness_box, Accessory.comma_power, Cable.long_obdc_cable])
  nissan_b = BaseCarHarness("Nissan B connector", parts=[Accessory.harness_box, Accessory.comma_power, Cable.long_obdc_cable])
  mazda = BaseCarHarness("Mazda connector")
  ford_q3 = BaseCarHarness("Ford Q3 connector")
  ford_q4 = BaseCarHarness("Ford Q4 connector", parts=[Accessory.harness_box, Accessory.comma_power, Cable.long_obdc_cable])
  rivian_a = BaseCarHarness("Rivian A connector", parts=[Accessory.harness_box, Accessory.comma_power, Cable.long_obdc_cable])
  rivian_b = BaseCarHarness("Rivian B connector", parts=[Accessory.harness_box, Accessory.comma_power, Cable.long_obdc_cable])
  tesla_a = BaseCarHarness("Tesla A connector", parts=[Accessory.harness_box, Cable.long_obdc_cable])
  tesla_b = BaseCarHarness("Tesla B connector", parts=[Accessory.harness_box, Cable.long_obdc_cable])
  psa_a = BaseCarHarness("PSA A connector", parts=[Accessory.harness_box, Cable.long_obdc_cable])


class Device(EnumBase):
  four = BasePart("comma four", parts=[Mount.mount, Cable.obd_c_cable_2ft])


class PartType(Enum):
  accessory = Accessory
  cable = Cable
  connector = CarHarness
  device = Device
  mount = Mount
  tool = Tool


DEFAULT_CAR_PARTS: list[EnumBase] = [Device.four]


@dataclass
class CarParts:
  parts: list[EnumBase] = field(default_factory=list)

  def __call__(self):
    return copy.deepcopy(self)

  @classmethod
  def common(cls, add: list[EnumBase] | None = None, remove: list[EnumBase] | None = None):
    p = [part for part in (add or []) + DEFAULT_CAR_PARTS if part not in (remove or [])]
    return cls(p)

  def all_parts(self):
    parts = []
    for part in self.parts:
      parts.extend(part.value.all_parts())
    return self.parts + parts


CarFootnote = namedtuple("CarFootnote", ["text", "column", "docs_only", "setup_note"], defaults=(False, False))


class CommonFootnote(Enum):
  EXP_LONG_AVAIL = CarFootnote(
    "openpilot Longitudinal Control (Alpha) is available behind a toggle; " +
    "the toggle is only available in non-release branches such as `nightly-dev`.",
    Column.LONGITUDINAL, docs_only=True)


def get_footnotes(footnotes: list[Enum], column: Column) -> list[Enum]:
  # Returns applicable footnotes given current column
  return [fn for fn in footnotes if fn.value.column == column]


# TODO: store years as a list
def get_year_list(years):
  years_list = []
  if len(years) == 0:
    return years_list

  for year in years.split(','):
    year = year.strip()
    if len(year) == 4:
      years_list.append(str(year))
    elif "-" in year and len(year) == 7:
      start, end = year.split("-")
      years_list.extend(map(str, range(int(start), int(f"20{end}") + 1)))
    else:
      raise Exception(f"Malformed year string: {years}")
  return years_list


def split_name(name: str) -> tuple[str, str, str]:
  make, model = name.split(" ", 1)
  years = ""
  match = re.search(MODEL_YEARS_RE, model)
  if match is not None:
    years = model[match.start():]
    model = model[:match.start() - 1]
  return make, model, years


SETUP = {"title": "comma four setup", "url": "https://comma.ai/setup"}
SETUP_VIDEO = "https://www.youtube.com/watch?v=Us4FodlaQ74"
WIKI = "https://github.com/commaai/openpilot/wiki/"


def source(title, url):
  return {"title": title, "url": url}


def step(step_id, title, body, media, sources=None):
  return {"id": step_id, "title": title, "body": body, "media": media, "sources": sources or [dict(SETUP)]}


def mount_step():
  return step("mount", "Place mount high and centered on the windshield", [
    "Clean the mounting area with the included alcohol wipe and let it dry.",
    "Leave enough room for the cable when the device is mounted, and make sure you can see the screen from a seated position.",
    "Center the mount horizontally so the device can properly center in the lane.",
    "Peel off the mount's tape cover. Press firmly in the center, then the edges. Make sure no bubbles form.",
  ], "mount")


def device_step(remote=False):
  body = [
    "Plug the OBD-C cable into the harness box. Make sure the cable is fully seated.",
    "Plug the OBD-C cable into the comma four. Make sure the cable is all the way in.",
    "Slide the device onto the mount until it is fully seated.",
  ]
  if remote:
    body.insert(0, "Route the long OBD-C cable from the harness box to the windshield. Keep it clear of pedals, moving parts, and airbag deployment paths.")
  return step("device", "Plug in OBD-C and mount the device", body, "device-remote" if remote else "device")


def camera_steps():
  return [
    step("access", "Remove the rearview mirror cover trim", [
      "Check the cover release method for your model and year before removing it. Keep the camera attached to the windshield.",
      "The photos show an example installation; covers and connector positions vary by vehicle.",
    ], "camera-access"),
    step("connect", "Connect car harness into the camera", [
      "Unplug the existing LKAS camera connector and plug it into the harness.",
      "Connect the other end of the harness back into the LKAS camera.",
      "Connect the small car-harness connector to the harness box and make sure it is fully seated.",
      "Find a spot for the harness box so the trim can still fit over it. Peel off the adhesive backing and attach the box in place.",
    ], "camera-connect"),
    mount_step(), device_step(),
    step("close", "Reinstall the rearview mirror cover trim", [
      "Make sure the car harness fully fits inside the trim.",
      "Generally, it is best to have the OBD-C cable come out of the top.",
    ], "camera-close"),
  ]


def remote_steps(location, access, module, media, sources):
  return [
    step("access", f"Access the {location}", access, media, sources),
    step("connect", f"Connect the harness at the {module}", [
      f"Unplug the vehicle connector at the {module} and plug it into the matching end of your car harness.",
      f"Plug the other end of the harness back into the {module}. Make sure the connector locks are fully engaged.",
      "Connect the small car-harness connector to the harness box and make sure it is fully seated.",
      "Secure the harness box where the trim can fit back in place without pinching any cables.",
    ], media, sources),
    mount_step(), device_step(remote=True),
    step("close", "Secure the cable and reinstall the trim", [
      "Secure excess cable so it cannot move into pedals, seat rails, or other moving parts. Keep airbag deployment paths clear.",
      "Check that the harness and cable fit behind the panels, then reinstall the trim you removed.",
    ], media, sources),
  ]


def get_default_setup_guide() -> dict:
  """Return independent JSON-ready default installation content."""
  guide = {
    "id": "default", "title": "comma four setup", "summary": "Install your comma four in five steps.",
    "steps": camera_steps(), "power": "optional", "notes": [], "power_notes": [],
    "power_instructions": [
      "Plug comma power into the vehicle's OBD-II port. Route its cable clear of pedals and airbag deployment paths.",
      "Connect its small 4-pin plug to the matching socket on the harness box.",
    ],
    "parts": ["comma four", "mount", "OBD-C cable", "car harness", "harness box", "comma power"],
    "sources": [dict(SETUP)], "video": SETUP_VIDEO, "video_is_reference": False,
  }
  return guide


def apply_remote_setup(guide, harness, location, access, module, media, sources):
  guide["steps"] = remote_steps(location, access, module, media, sources)
  guide["summary"] = f"Your {harness.value.name} connects at the {module}. Follow the vehicle-specific steps below."
  guide["sources"] = [dict(SETUP), *sources]
  guide["video_is_reference"] = True


@dataclass
class CarDocs:
  # make + model + model years
  name: str

  # the simplest description of the requirements for the US market
  package: str

  video: str | None = None
  setup_video: str | None = None
  footnotes: list[Enum] = field(default_factory=list)
  min_steer_speed: float | None = None
  min_enable_speed: float | None = None
  auto_resume: bool | None = None

  # all the parts needed for the supported car
  car_parts: CarParts = field(default_factory=CarParts)
  # Mazda requires comma power
  requires_comma_power: bool = False

  merged: bool = True
  support_type: SupportType = SupportType.UPSTREAM
  support_link: str | None = "#upstream"

  def __post_init__(self):
    self.make, self.model, self.years = split_name(self.name)
    self.year_list = get_year_list(self.years)

  def init_setup(self, guide):
    """Customize installation content in the brand's CarDocs subclass."""

  def get_setup_guide(self) -> dict:
    """Build independent installation content for this initialized CarDocs entry."""
    guide = get_default_setup_guide()
    parts = self.car_parts.all_parts()
    harness = next((p for p in parts if isinstance(p, CarHarness)), None)
    if harness is None:
      raise ValueError(f"No installation harness for {self.name}")
    part_names = ("supplied short OBD-C cable" if p == Cable.obd_c_cable_2ft else p.value.name for p in parts)
    guide.update(id=self.name, title=f"{self.make} {self.model}", parts=list(dict.fromkeys(part_names)))
    guide["video"] = self.setup_video or SETUP_VIDEO
    guide["summary"] = f"Install your comma four with the {harness.value.name}."
    guide["sources"].append(source(f"{self.make} installation reference", WIKI + getattr(self, "brand", "")))
    if self.setup_video:
      guide["sources"].append(source("Vehicle installation video", self.setup_video))
    self.init_setup(guide)
    if guide["video"] != SETUP_VIDEO and not any(s["url"] == guide["video"] for s in guide["sources"]):
      guide["sources"].append(source("Vehicle installation video", guide["video"]))
    if self.requires_comma_power:
      guide["power"] = "required"
      guide["steps"].insert(2, step("power", "Connect comma power", [
        *guide["power_instructions"],
        "Your vehicle needs this power connection. Route the cable clear of pedals and airbag deployment paths.",
      ], "power", guide["sources"][1:]))
    elif Accessory.comma_power not in parts:
      guide["power"] = "included"
    # CAN networkLocation does not describe physical access or the video's hardware generation.
    guide["video_is_reference"] = True
    return guide

  def init(self, CP: CarParams, all_footnotes=None):
    self.brand = CP.brand
    self.car_fingerprint = CP.carFingerprint
    self.longitudinal_control = CP.openpilotLongitudinalControl and not CP.alphaLongitudinalAvailable

    if self.merged and CP.dashcamOnly:
      if self.support_type not in (SupportType.CUSTOM, SupportType.REVIEW):
        self.support_type = SupportType.DASHCAM
        self.support_link = "#dashcam"

    # longitudinal column
    op_long = "Stock"
    if CP.alphaLongitudinalAvailable:
      op_long = "openpilot available"
      self.footnotes.append(CommonFootnote.EXP_LONG_AVAIL)
    elif CP.openpilotLongitudinalControl:
      op_long = "openpilot"

    # min steer & enable speed columns
    # TODO: set all the min steer speeds in carParams and remove this
    if self.min_steer_speed is not None:
      assert CP.minSteerSpeed < 0.5, f"{CP.carFingerprint}: Minimum steer speed set in both CarDocs and CarParams"
    else:
      self.min_steer_speed = CP.minSteerSpeed

    # TODO: set all the min enable speeds in carParams correctly and remove this
    if self.min_enable_speed is None:
      self.min_enable_speed = CP.minEnableSpeed

    if self.auto_resume is None:
      self.auto_resume = CP.autoResumeSng and self.min_enable_speed <= 0

    # hardware column
    hardware_col = "None"
    if self.car_parts.parts:
      buy_link = f'<a href="https://comma.ai/shop/comma-3x?harness={self.name}">Buy Here</a>'

      tools_docs = [part for part in self.car_parts.all_parts() if isinstance(part, Tool)]
      parts_docs = [part for part in self.car_parts.all_parts() if not isinstance(part, Tool)]

      def display_func(parts):
        return '<br>'.join([f"- {parts.count(part)} {part.value.name}" for part in sorted(set(parts), key=lambda part: str(part.value.name))])

      hardware_col = f'<details><summary>Parts</summary><sub>{display_func(parts_docs)}<br>{buy_link}</sub></details>'
      if len(tools_docs):
        hardware_col += f'<details><summary>Tools</summary><sub>{display_func(tools_docs)}</sub></details>'

    self.row: dict[Enum, str | Star] = {
      Column.MAKE: self.make,
      Column.MODEL: self.model,
      Column.PACKAGE: self.package,
      Column.LONGITUDINAL: op_long,
      Column.FSR_LONGITUDINAL: f"{max(self.min_enable_speed * CV.MS_TO_MPH, 0):.0f} mph",
      Column.FSR_STEERING: f"{max(self.min_steer_speed * CV.MS_TO_MPH, 0):.0f} mph",
      Column.STEERING_TORQUE: Star.EMPTY,
      Column.AUTO_RESUME: Star.FULL if self.auto_resume else Star.EMPTY,
      Column.HARDWARE: hardware_col,
      Column.VIDEO: self.video or "",  # replaced with an image and link from template in get_column
      Column.SETUP_VIDEO: self.setup_video or "",  # replaced with an image and link from template in get_column
    }

    if self.support_link is not None:
      support_info = f"[{self.support_type.value}]({self.support_link})"
    else:
      support_info = self.support_type.value

    self.extra_cars_row: dict[Enum, str] = {
      ExtraCarsColumn.MAKE: self.make,
      ExtraCarsColumn.MODEL: self.model,
      ExtraCarsColumn.PACKAGE: self.package,
      ExtraCarsColumn.SUPPORT: support_info,
    }

    # Set steering torque star from max lateral acceleration
    assert CP.maxLateralAccel > 0.1
    if CP.maxLateralAccel >= GOOD_TORQUE_THRESHOLD:
      self.row[Column.STEERING_TORQUE] = Star.FULL

    self.all_footnotes = all_footnotes
    self.detail_sentence = self.get_detail_sentence(CP)

    return self

  def init_make(self, CP: CarParams):
    """CarDocs subclasses can add make-specific logic for harness selection, footnotes, etc."""

  def get_detail_sentence(self, CP):
    if not CP.notCar:
      sentence_builder = "openpilot upgrades your <strong>{car_model}</strong> with automated lane centering{alc} and adaptive cruise control{acc}."

      if self.min_steer_speed > self.min_enable_speed:
        alc = f" <strong>above {self.min_steer_speed * CV.MS_TO_MPH:.0f} mph</strong>," if self.min_steer_speed > 0 else " <strong>at all speeds</strong>,"
      else:
        alc = ""

      # Exception for cars which do not auto-resume yet
      acc = ""
      if self.min_enable_speed > 0:
        acc = f" <strong>while driving above {self.min_enable_speed * CV.MS_TO_MPH:.0f} mph</strong>"
      elif self.auto_resume:
        acc = " <strong>that automatically resumes from a stop</strong>"

      if self.row[Column.STEERING_TORQUE] != Star.FULL:
        sentence_builder += " This car may not be able to take tight turns on its own."

      # experimental mode
      exp_link = "<a href='https://blog.comma.ai/090release/#experimental-mode' target='_blank' class='highlight'>Experimental mode</a>"
      if CP.openpilotLongitudinalControl and not CP.alphaLongitudinalAvailable and CP.autoResumeSng:
        sentence_builder += f" Traffic light and stop sign handling is also available in {exp_link}."

      return sentence_builder.format(car_model=f"{self.make} {self.model}", alc=alc, acc=acc)

    else:
      if CP.carFingerprint == "COMMA_BODY":
        return "The body is a robotics dev kit that can run openpilot. <a href='https://www.commabody.com' target='_blank' class='highlight'>Learn more.</a>"
      else:
        raise Exception(f"This notCar does not have a detail sentence: {CP.carFingerprint}")

  def get_column(self, column: Column, star_icon: str, video_icon: str, footnote_tag: str) -> str:
    item: str | Star = self.row[column]
    if isinstance(item, Star):
      item = star_icon.format(item.value)
    elif column == Column.MODEL and len(self.years):
      item += f" {self.years}"
    elif column in (Column.VIDEO, Column.SETUP_VIDEO) and len(item) > 0:
      item = video_icon.format(item)

    footnotes = get_footnotes(self.footnotes, column)
    if len(footnotes):
      sups = sorted([self.all_footnotes[fn] for fn in footnotes])
      item += footnote_tag.format(f'{",".join(map(str, sups))}')

    return item

  def get_extra_cars_column(self, column: ExtraCarsColumn) -> str:
    item: str = self.extra_cars_row[column]
    if column == ExtraCarsColumn.MODEL and len(self.years):
      item += f" {self.years}"

    return item


@dataclass
class ExtraCarDocs(CarDocs):
  package: str = "All"
  merged: bool = False
  support_type: SupportType = SupportType.INCOMPATIBLE
  support_link: str | None = "#incompatible"
