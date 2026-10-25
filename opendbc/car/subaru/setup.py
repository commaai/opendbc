from opendbc.car.docs_definitions import WIKI, source, step


def configure_setup(car, guide):
  refs = [source(f"{car.make} installation reference", WIKI + 'Subaru')]
  if car.setup_video:
    refs.append(source("Vehicle installation video", car.setup_video))
  guide["sources"][1:] = refs
  steps = guide["steps"]
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
  guide["video_is_reference"] = True
