from opendbc.car.docs_definitions import WIKI, source, step, CarHarness, apply_remote_setup


def configure_setup(car, guide):
  refs = [source(f"{car.make} installation reference", WIKI + 'GM')]
  if car.setup_video:
    refs.append(source("Vehicle installation video", car.setup_video))
  guide["sources"][1:] = refs
  guide["steps"][0] = step("access", 'Remove the camera cover', [
    'Remove the cover behind the rearview mirror to expose the front camera connector.',
    'Leave the camera mounted to the windshield.',
  ], "camera-access", refs)
  harness = next(p for p in car.car_parts.all_parts() if isinstance(p, CarHarness))
  remote = None
  if harness == CarHarness.gmsdgm:
    refs = [source("GM installation", WIKI + "GM")]
    remote = ("serial data gateway", [
      "Locate the serial data gateway using the installation reference for your vehicle and remove the surrounding trim.",
      "The GM SDGM harness connects at this gateway rather than the windshield camera.",
    ], "serial data gateway", "footwell")
  if remote:
    apply_remote_setup(guide, harness, *remote, refs)
