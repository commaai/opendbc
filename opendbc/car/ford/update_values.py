
path = "values.py"
content = open(path, encoding="utf-8").read()

if "SECOC =" not in content:
    content = content.replace("class FordFlags(IntFlag):", "class FordFlags(IntFlag):\n  SECOC = 4  # SecOC / TRON")

if "FordSecOCPlatformConfig" not in content:
    secoc_config = "\n@dataclass\nclass FordSecOCPlatformConfig(FordCANFDPlatformConfig):\n  def init(self):\n    super().init()\n    self.flags |= FordFlags.SECOC\n"
    content = content.replace("class FordF150LightningPlatform(FordCANFDPlatformConfig):", secoc_config + "\nclass FordF150LightningPlatform(FordCANFDPlatformConfig):")

if "FORD_F_150_MK15" not in content:
    f150_2026 = "  FORD_F_150_MK15 = FordSecOCPlatformConfig(\n    [FordCarDocs(\"Ford F-150 2024-26\", \"Co-Pilot360 Assist 2.0\", hybrid=True)],\n    CarSpecs(wheelbase=3.69, steerRatio=17.0),\n  )\n"
    content = content.replace("  FORD_F_150_MK14 = FordPlatformConfig(", f150_2026 + "  FORD_F_150_MK14 = FordPlatformConfig(")

open(path, "w", encoding="utf-8").write(content)
print("Successfully updated values.py!")

