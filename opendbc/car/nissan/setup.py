from opendbc.car.docs_definitions import WIKI, source, CarHarness, apply_remote_setup


def configure_setup(car, guide):
  refs = [source(f"{car.make} installation reference", WIKI + 'nissan')]
  if car.setup_video:
    refs.append(source("Vehicle installation video", car.setup_video))
  guide["sources"][1:] = refs
  harness = next(p for p in car.car_parts.all_parts() if isinstance(p, CarHarness))
  remote = None
  if harness in (CarHarness.nissan_a, CarHarness.nissan_b):
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
  if remote:
    apply_remote_setup(guide, harness, *remote, refs)
