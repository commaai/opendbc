from opendbc.car.docs_definitions import WIKI, source, step


def configure_setup(car, guide):
  refs = [source(f"{car.make} installation reference", WIKI + 'Toyota-Lexus')]
  if car.setup_video:
    refs.append(source("Vehicle installation video", car.setup_video))
  guide["sources"][1:] = refs
  guide["steps"][0] = step("access", 'Remove the camera cover', [
    'Slide off the cover around the camera behind the rearview mirror to expose its connector.',
    'Keep the camera attached to the windshield while fitting the harness.',
  ], "camera-access", refs)
