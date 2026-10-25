from opendbc.car.docs_definitions import WIKI, source, step


def configure_setup(car, guide):
  refs = [source(f"{car.make} installation reference", WIKI + 'FCA')]
  if car.setup_video:
    refs.append(source("Vehicle installation video", car.setup_video))
  guide["sources"][1:] = refs
  guide["steps"][0] = step("access", 'Remove the camera cover', [
    'Remove the cover behind the rearview mirror to reach the driver-assistance camera.',
    'Leave the camera and mirror attached while you install the harness.',
  ], "camera-access", refs)
