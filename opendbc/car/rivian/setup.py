from opendbc.car.docs_definitions import WIKI, source, CarHarness, apply_remote_setup


def configure_setup(car, guide):
  refs = [source(f"{car.make} installation reference", WIKI + 'rivian')]
  if car.setup_video:
    refs.append(source("Vehicle installation video", car.setup_video))
  guide["sources"][1:] = refs
  harness = next(p for p in car.car_parts.all_parts() if isinstance(p, CarHarness))
  remote = None
  if harness in (CarHarness.rivian_a, CarHarness.rivian_b):
    refs = [source("Rivian installation", WIKI + "Rivian"),
            source("Rivian Gen 1 preparation and installation",
                   "https://docs.google.com/document/d/1clodGRssTx-dSPSUrQITq_vqumgWcpSUw83zdcAZR-A/edit")]
    remote = ("AXM in the passenger footwell", [
      "Before removing trim or unplugging the module, follow the vehicle sleep procedure in the Rivian Gen 1 preparation reference.",
      "Remove the passenger footwell trim to reach the AXM driver-assistance module and identify its connector using the reference photos.",
    ], "AXM", "footwell")
  if remote:
    apply_remote_setup(guide, harness, *remote, refs)
