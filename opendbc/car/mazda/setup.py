from opendbc.car.docs_definitions import WIKI, source, step


def configure_setup(car, guide):
  refs = [source(f"{car.make} installation reference", WIKI + 'Mazda')]
  if car.setup_video:
    refs.append(source("Vehicle installation video", car.setup_video))
  guide["sources"][1:] = refs
  guide["steps"][0] = step("access", 'Remove the camera cover', [
    'Separate the cover around the rearview mirror to expose the forward-facing camera connector.',
    'This installation also needs comma power, connected in the power step below.',
  ], "camera-access", refs)
