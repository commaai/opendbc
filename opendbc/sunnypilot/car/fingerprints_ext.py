"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""


def merge_fw_versions(fw_versions, new_fw_versions):
  """
    Merge firmware versions by extending lists for matching ECUs,
    adding all entries regardless of duplicates.
  """
  for c, f in new_fw_versions.items():
    if c not in fw_versions:
      fw_versions[c] = f
      continue

    for e, new_fw_list in f.items():
      if e not in fw_versions[c]:
        fw_versions[c][e] = new_fw_list
      else:
        fw_versions[c][e].extend(new_fw_list)

  return fw_versions


def merge_fingerprints(fingerprints, new_fingerprints):
  """
    Merge fingerprints by extending lists for matching keys,
    adding all entries regardless of duplicates.
  """
  for car, fp_list in new_fingerprints.items():
    if car not in fingerprints:
      fingerprints[car] = fp_list
    else:
      fingerprints[car].extend(fp_list)

  return fingerprints
