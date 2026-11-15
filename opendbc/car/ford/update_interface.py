
# 1. Update FordSafetyFlags in values.py
val_path = "values.py"
val_content = open(val_path, encoding="utf-8").read()
if "SECOC = 4" not in val_content:
    val_content = val_content.replace("class FordSafetyFlags(IntFlag):\n  LONG_CONTROL = 1\n  CANFD = 2", "class FordSafetyFlags(IntFlag):\n  LONG_CONTROL = 1\n  CANFD = 2\n  SECOC = 4")
    open(val_path, "w", encoding="utf-8").write(val_content)

# 2. Update interface.py to support SECOC
int_path = "interface.py"
int_content = open(int_path, encoding="utf-8").read()
old_block = """      # TRON (SecOC) platforms are not supported
      # LateralMotionControl2, ACCDATA are 16 bytes on these platforms
      if len(fingerprint[CAN.camera]):
        if fingerprint[CAN.camera].get(0x3d6) != 8 or fingerprint[CAN.camera].get(0x186) != 8:
          carlog.error(\x27dashcamOnly: SecOC is unsupported\x27)
          ret.dashcamOnly = True"""

new_block = """      if ret.flags & FordFlags.SECOC:
        ret.secOcRequired = True
        ret.safetyConfigs[-1].safetyParam |= FordSafetyFlags.SECOC.value
      else:
        # TRON (SecOC) platforms are not supported
        # LateralMotionControl2, ACCDATA are 16 bytes on these platforms
        if len(fingerprint[CAN.camera]):
          if fingerprint[CAN.camera].get(0x3d6) != 8 or fingerprint[CAN.camera].get(0x186) != 8:
            carlog.error(\x27dashcamOnly: SecOC is unsupported\x27)
            ret.dashcamOnly = True"""

if "ret.secOcRequired = True" not in int_content:
    int_content = int_content.replace(old_block, new_block)
    open(int_path, "w", encoding="utf-8").write(int_content)

print("Successfully updated interface.py and safety flags!")

