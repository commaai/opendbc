from opendbc.car.docs_definitions import WIKI, source, CarHarness, apply_remote_setup


def configure_setup(car, guide):
  refs = [source(f"{car.make} installation reference", WIKI + 'volkswagen')]
  if car.setup_video:
    refs.append(source("Vehicle installation video", car.setup_video))
  guide["sources"][1:] = refs
  harness = next(p for p in car.car_parts.all_parts() if isinstance(p, CarHarness))
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
  if remote:
    apply_remote_setup(guide, harness, *remote, refs)
    if harness == CarHarness.vw_j533:
      guide["steps"][0]["body"].insert(0, "Turn the vehicle off before disconnecting the gateway.")
      guide["steps"][1]["body"].insert(0,
        "Plug the harness's integrated RJ45 lead into the harness box if disconnected. Check that the harness connector is seated in the box too.")
