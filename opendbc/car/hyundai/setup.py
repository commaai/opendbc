from opendbc.car.docs_definitions import WIKI, source, step


def configure_setup(car, guide):
  refs = [source(f"{car.make} installation reference", WIKI + 'Hyundai-Kia-Genesis')]
  if car.setup_video:
    refs.append(source("Vehicle installation video", car.setup_video))
  guide["sources"][1:] = refs
  guide["steps"][0] = step("access", 'Remove the camera cover', [
    'Remove the cover behind the rearview mirror to expose the forward-facing camera connector.',
    'Use the harness selected for your vehicle; HDA II and other camera systems use different connectors.',
  ], "camera-access", refs)
