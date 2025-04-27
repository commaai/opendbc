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
