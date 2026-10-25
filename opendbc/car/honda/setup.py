from opendbc.car.docs_definitions import WIKI, source, step


def configure_setup(car, guide):
  refs = [source(f"{car.make} installation reference", WIKI + 'Honda-Acura')]
  if car.setup_video:
    refs.append(source("Vehicle installation video", car.setup_video))
  guide["sources"][1:] = refs
  guide["steps"][0] = step("access", 'Remove the camera cover', [
    'Release the plastic cover behind the rearview mirror to reach the driver-assistance camera.',
    "Release the cover's clips carefully; leave the camera attached to the windshield.",
  ], "camera-access", refs)
