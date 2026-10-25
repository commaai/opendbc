"""Vehicle installation content for comma four. Presentation and images belong to consumers.

Guides deliberately share mounting/device steps. Harness and model determine the
connection location; a remote harness must never inherit camera instructions.
Source links document access procedures, including videos made with older devices.
"""

from opendbc.car.docs_definitions import Accessory, CarDocs, CarHarness, SupportType

SETUP = {"title": "comma four setup", "url": "https://comma.ai/setup"}
WIKI = "https://github.com/commaai/openpilot/wiki/"


def source(title, url):
  return {"title": title, "url": url}


def step(step_id, title, body, media, sources=None):
  return {"id": step_id, "title": title, "body": body, "media": media, "sources": sources or [SETUP]}


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
      "Removal method varies by car. The plastic often needs a firm tug on Hondas; Toyotas often need a slide outward.",
      "Use a vehicle-specific installation video to check how the cover releases.",
    ], "camera-access"),
    step("connect", "Connect car harness into the camera", [
      "Unplug the existing LKAS camera connector and plug it into the harness.",
      "Connect the other end of the harness back into the LKAS camera.",
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
      "Secure the harness box where the trim can fit back in place without pinching any cables.",
    ], media, sources),
    mount_step(), device_step(remote=True),
    step("close", "Secure the cable and reinstall the trim", [
      "Secure excess cable so it cannot move into pedals, seat rails, or other moving parts. Keep airbag deployment paths clear.",
      "Check that the harness and cable fit behind the panels, then reinstall the trim you removed.",
    ], media, sources),
  ]


# Changes to the camera access procedure, rather than separate copies of the guide.
CAMERA_ACCESS = {
  "toyota": ("Remove the camera cover", ["Slide off the cover around the camera behind the rearview mirror to expose its connector.",
                                          "Keep the camera attached to the windshield while fitting the harness."]),
  "honda": ("Remove the camera cover", ["Release the plastic cover behind the rearview mirror to reach the driver-assistance camera.",
                                         "Release the cover's clips carefully; leave the camera attached to the windshield."]),
  "hyundai": ("Remove the camera cover", ["Remove the cover behind the rearview mirror to expose the forward-facing camera connector.",
                                           "Use the harness selected for your vehicle; HDA II and other camera systems use different connectors."]),
  "chrysler": ("Remove the camera cover", ["Remove the cover behind the rearview mirror to reach the driver-assistance camera.",
                                            "Leave the camera and mirror attached while you install the harness."]),
  "gm": ("Remove the camera cover", ["Remove the cover behind the rearview mirror to expose the front camera connector.",
                                      "Leave the camera mounted to the windshield."]),
  "mazda": ("Remove the camera cover", ["Separate the cover around the rearview mirror to expose the forward-facing camera connector.",
                                         "This installation also needs comma power, connected in the power step below."]),
  "ford": ("Remove the camera cover", ["Remove the plastic cover behind the rearview mirror to expose the camera connector.",
                                        "The Ford Q3 harness connects here at the camera."]),
}


def get_setup_guide(car: CarDocs | None = None) -> dict:
  """Return independent JSON-ready content for one initialized CarDocs entry."""
  guide = {
    "id": "default", "title": "comma four setup", "summary": "Install your comma four in five steps.",
    "steps": camera_steps(), "power": "optional",
    "parts": ["comma four", "mount", "OBD-C cable", "car harness", "harness box", "comma power"],
    "sources": [SETUP],
  }
  if car is None:
    return guide

  parts = car.car_parts.all_parts()
  harness = next((p for p in parts if isinstance(p, CarHarness)), None)
  if harness is None:
    raise ValueError(f"No installation harness for {car.name}")
  brand = getattr(car, "brand", "")
  guide.update(id=car.name, title=f"{car.make} {car.model}", parts=list(dict.fromkeys(p.value.name for p in parts)))
  if car.setup_video:
    guide["video"] = car.setup_video
  guide["summary"] = f"Install your comma four with the {harness.value.name}."
  wiki_page = {"honda": "Honda-Acura", "toyota": "Toyota-Lexus", "chrysler": "FCA",
               "hyundai": "Hyundai-Kia-Genesis", "gm": "GM", "subaru": "Subaru", "mazda": "Mazda"}.get(brand, brand)
  refs = [source(f"{car.make} installation reference", WIKI + wiki_page)]
  if car.setup_video:
    refs.append(source("Vehicle installation video", car.setup_video))
  guide["sources"] = [SETUP, *refs]
  steps = guide["steps"]

  if brand in CAMERA_ACCESS:
    title, body = CAMERA_ACCESS[brand]
    steps[0] = step("access", title, list(body), "camera-access", refs)
  if brand == "subaru":
    steps[0] = step("access", "Remove the EyeSight cover", [
      "Remove the overhead EyeSight cover to reach its wiring connector. Use a trim tool and the socket listed in your parts.",
      "Leave the EyeSight cameras in place and avoid touching their lenses.",
    ], "overhead", refs)
    steps[1]["title"] = "Connect the harness to EyeSight"
    steps[1]["body"][0] = "Disconnect the EyeSight wiring connector and plug it into the matching end of the Subaru harness."
    steps[1]["body"][1] = "Connect the other end of the harness back to EyeSight."
    steps[1]["media"] = "overhead"
    steps[4] = step("close", "Reinstall the EyeSight cover", [
      "Secure the harness clear of the EyeSight cameras and make sure the cover fits without pinching the cable.",
      "Route the OBD-C cable toward the comma four and reinstall the cover.",
    ], "overhead", refs)

  remote = None
  if harness in (CarHarness.vw_j533, CarHarness.vw_meb):
    refs = [source("Volkswagen gateway installation", "https://docs.howtocomma.com/docs/j533-harness-install")]
    if harness == CarHarness.vw_meb:
      refs = [source("Volkswagen MEB installation", WIKI + "Volkswagen")]
      remote = ("MEB gateway", [
        "Locate the MEB gateway using the installation reference for your vehicle before removing trim.",
        "Use your VW MEB connector at the gateway. The J533 connector is a different harness.",
      ], "MEB gateway", "module")
    elif car.model.startswith(("Caravelle", "California")):
      remote = ("gateway behind the instrument cluster", [
        "Remove the instrument-cluster trim to reach the J533 gateway behind the cluster.",
        "Use the gateway reference to identify the module before unplugging its connector.",
      ], "J533 gateway", "cluster")
    else:
      remote = ("gateway under the dashboard", [
        "Remove the lower dashboard trim in the driver's footwell to reach the gateway.",
        "Identify the gateway and release its connector locking mechanism before unplugging it.",
      ], "gateway", "footwell")
    guide["power"] = "included"
  elif harness == CarHarness.ford_q4:
    refs = [source("Ford installation locations", "https://github.com/BluePilotDev/bluepilot#-installation")]
    videos = {"Mustang Mach-E": "AR4_eTF3b_A", "Escape": "M6uXf4b2SHM", "Ranger": "2oJlXCKYOy0"}
    for model, video_id in videos.items():
      if car.model.startswith(model):
        guide["video"] = "https://www.youtube.com/watch?v=" + video_id
    if car.model.startswith(("F-150", "Expedition")):
      remote = ("IPMA in the driver's footwell", ["Remove the driver's lower dash panel to reach the IPMA module above the pedals."], "IPMA", "footwell")
    elif car.model.startswith(("Mustang Mach-E", "Escape")):
      remote = ("IPMA in the cargo area", ["Open the cargo area and remove the trim covering the IPMA module.",
                                             "Use the vehicle installation reference to identify the IPMA before disconnecting it."], "IPMA", "cargo")
    elif car.model.startswith("Ranger"):
      remote = ("IPMA behind the instrument cluster", ["Remove the instrument-cluster trim to reach the IPMA behind the cluster."], "IPMA", "cluster")
    else:
      remote = ("IPMA module", ["Locate the separate IPMA module using the installation reference for your vehicle.",
                                 "The Q4 harness connects at this module; its location varies by model and market."], "IPMA", "module")
  elif harness in (CarHarness.nissan_a, CarHarness.nissan_b):
    refs = [source("Nissan ADAS installation locations", WIKI + "nissan")]
    if car.model.startswith("Leaf"):
      remote = ("ADAS module behind the glovebox", [
        "Remove the glovebox to look for the ADAS module above it.",
        "Some Leaf variants place the module behind the instrument cluster instead. Check the reference photos: other nearby connectors also fit the harness.",
      ], "ADAS module", "glovebox")
    elif car.model.startswith("Altima"):
      remote = ("ADAS module behind the glovebox", [
        "Look for the ADAS module behind the glovebox on the right side.",
        "Use the Nissan reference photos to identify the ADAS module before disconnecting its wiring.",
      ], "ADAS module", "glovebox")
    elif car.model.startswith("Rogue"):
      remote = ("ADAS module in the cargo area", [
        "Lift the luggage floor cover to expose the spare-tire area and locate the ADAS module.",
        "Check that your cable reaches the windshield before routing it behind trim.",
      ], "ADAS module", "cargo")
    else:
      remote = ("ADAS module", ["Locate the ADAS module using the Nissan installation reference for your model and market.",
                                  "The Nissan harness connects to this separate module, not the windshield camera."], "ADAS module", "module")
  elif harness in (CarHarness.tesla_a, CarHarness.tesla_b):
    refs = [source("Tesla HW3 / HW4 installation", "https://wiki.xnor.shop/docs/tesla/hw3-hw4/")]
    remote = ("Autopilot computer behind the glovebox", [
      "Follow the Tesla reference to power down the vehicle before disconnecting the Autopilot computer.",
      "Remove the passenger footwell trim to reach the Autopilot computer behind the glovebox.",
      "Use the Tesla connector supplied for the hardware generation in your selected vehicle.",
    ], "Autopilot computer", "glovebox")
    guide["power"] = "included"
  elif harness in (CarHarness.rivian_a, CarHarness.rivian_b):
    refs = [source("Rivian installation", WIKI + "Rivian"),
            source("Rivian Gen 1 preparation and installation",
                   "https://docs.google.com/document/d/1clodGRssTx-dSPSUrQITq_vqumgWcpSUw83zdcAZR-A/edit")]
    remote = ("AXM in the passenger footwell", [
      "Before removing trim or unplugging the module, follow the vehicle sleep procedure in the Rivian Gen 1 preparation reference.",
      "Remove the passenger footwell trim to reach the AXM driver-assistance module and identify its connector using the reference photos.",
    ], "AXM", "footwell")
  elif harness == CarHarness.gmsdgm:
    refs = [source("GM installation", WIKI + "GM")]
    remote = ("serial data gateway", [
      "Locate the serial data gateway using the installation reference for your vehicle and remove the surrounding trim.",
      "The GM SDGM harness connects at this gateway rather than the windshield camera.",
    ], "serial data gateway", "footwell")

  if remote:
    guide["steps"] = remote_steps(*remote, refs)
    if harness == CarHarness.vw_j533:
      guide["steps"][0]["body"].insert(0, "Turn the vehicle off before disconnecting the gateway.")
      guide["steps"][1]["body"].insert(0,
        "Plug the harness's integrated RJ45 lead into the harness box if disconnected. Check that the harness connector is seated in the box too.")
    guide["summary"] = f"Your {harness.value.name} connects at the {remote[2]}. Follow the vehicle-specific steps below."
  guide["sources"] = [SETUP, *refs]
  if guide.get("video") and not any(s["url"] == guide["video"] for s in guide["sources"]):
    guide["sources"].append(source("Vehicle installation video", guide["video"]))
  if car.requires_comma_power:
    guide["power"] = "required"
    guide["steps"].insert(2, step("power", "Connect comma power", [
      "Connect comma power to the vehicle's OBD-II port and connect its cable to the harness box.",
      "Your vehicle needs this power connection. Route the cable clear of pedals and airbag deployment paths.",
    ], "power", refs))
  elif Accessory.comma_power not in parts:
    guide["power"] = "included"
  return guide


def get_setup_guides(all_car_docs: list[CarDocs]) -> dict:
  """Match the website's supported-vehicle selector, excluding community/custom installs."""
  return {"default": get_setup_guide(), **{
    car.name: get_setup_guide(car) for car in all_car_docs
    if car.support_type == SupportType.UPSTREAM and car.name != "comma body"
  }}
