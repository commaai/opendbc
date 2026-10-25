from opendbc.car.docs_definitions import WIKI, source, step, CarHarness, apply_remote_setup


def configure_setup(car, guide):
  refs = [source(f"{car.make} installation reference", WIKI + 'ford')]
  if car.setup_video:
    refs.append(source("Vehicle installation video", car.setup_video))
  guide["sources"][1:] = refs
  guide["steps"][0] = step("access", 'Remove the camera cover', [
    'Remove the plastic cover behind the rearview mirror to expose the camera connector.',
    'The Ford Q3 harness connects here at the camera.',
  ], "camera-access", refs)
  harness = next(p for p in car.car_parts.all_parts() if isinstance(p, CarHarness))
  remote = None
  if harness == CarHarness.ford_q4:
    refs = [source("Ford installation locations", "https://github.com/BluePilotDev/bluepilot#-installation")]
    videos = {"Mustang Mach-E": "AR4_eTF3b_A", "Escape": "M6uXf4b2SHM", "Ranger": "2oJlXCKYOy0"}
    for model, video_id in videos.items():
      if car.model.startswith(model):
        guide["video"] = "https://www.youtube.com/watch?v=" + video_id
    if car.model.startswith(("F-150", "Expedition")):
      remote = ("IPMA in the driver's footwell", ["Remove the driver's lower dash panel to reach the IPMA module above the pedals."], "IPMA", "footwell")
    elif car.model.startswith(("Mustang Mach-E", "Escape")):
      remote = ("IPMA in the cargo area", ["Open the cargo area and remove the trim covering the IPMA module.",
                                             "Use the vehicle installation reference to identify the IPMA before disconnecting it."], "IPMA", "cargo")
    elif car.model.startswith("Ranger"):
      remote = ("IPMA behind the instrument cluster", ["Remove the instrument-cluster trim to reach the IPMA behind the cluster."], "IPMA", "cluster")
    else:
      remote = ("IPMA module", ["Locate the separate IPMA module using the installation reference for your vehicle.",
                                 "The Q4 harness connects at this module; its location varies by model and market."], "IPMA", "module")
  if remote:
    apply_remote_setup(guide, harness, *remote, refs)
