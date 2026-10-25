from opendbc.car.docs_definitions import WIKI, source, CarHarness, apply_remote_setup


def configure_setup(car, guide):
  refs = [source(f"{car.make} installation reference", WIKI + 'tesla')]
  if car.setup_video:
    refs.append(source("Vehicle installation video", car.setup_video))
  guide["sources"][1:] = refs
  harness = next(p for p in car.car_parts.all_parts() if isinstance(p, CarHarness))
  remote = None
  if harness in (CarHarness.tesla_a, CarHarness.tesla_b):
    refs = [source("Tesla HW3 / HW4 installation", "https://wiki.xnor.shop/docs/tesla/hw3-hw4/")]
    remote = ("Autopilot computer behind the glovebox", [
      "Follow the Tesla reference to power down the vehicle before disconnecting the Autopilot computer.",
      "Remove the passenger footwell trim to reach the Autopilot computer behind the glovebox.",
      "Use the Tesla connector supplied for the hardware generation in your selected vehicle.",
    ], "Autopilot computer", "glovebox")
    guide["power"] = "included"
  if remote:
    apply_remote_setup(guide, harness, *remote, refs)
