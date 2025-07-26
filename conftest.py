# import time
collect_ignore_glob = [
  "opendbc/safety/tests/misra/*.sh",
  "opendbc/safety/tests/misra/cppcheck/",
]

# from concurrent.futures import ThreadPoolExecutor
# import xdist.workermanage as wm
#
# def setup_nodes_concurrent(self, putevent):
#     self.config.hook.pytest_xdist_setupnodes(config=self.config, specs=self.specs)
#     self.trace("setting up nodes (concurrent)")
#     t = time.monotonic()
#     with ThreadPoolExecutor(max_workers=len(self.specs)) as pool:
#         futs = [pool.submit(self.setup_node, spec, putevent) for spec in self.specs]
#         ret = [f.result() for f in futs]
#         print(f"setup_nodes_concurrent took {time.monotonic() - t:.3f} seconds")
#         return ret
#
# wm.NodeManager.setup_nodes = setup_nodes_concurrent
