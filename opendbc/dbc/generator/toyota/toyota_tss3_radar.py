#!/usr/bin/env python3


def generate():
  parts = ["""
VERSION ""


NS_ :
    NS_DESC_
    CM_
    BA_DEF_
    BA_
    VAL_
    CAT_DEF_
    CAT_
    FILTER
    BA_DEF_DEF_
    EV_DATA_
    ENVVAR_DATA_
    SGTYPE_
    SGTYPE_VAL_
    BA_DEF_SGTYPE_
    BA_SGTYPE_
    SIG_TYPE_REF_
    VAL_TABLE_
    SIG_GROUP_
    SIG_VALTYPE_
    SIGTYPE_VALTYPE_
    BO_TX_BU_
    BA_DEF_REL_
    BA_REL_
    BA_DEF_DEF_REL_
    BU_SG_REL_
    BU_EV_REL_
    BU_BO_REL_
    SG_MUL_VAL_

BS_:

BU_: XXX
"""]

  header = """ SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" XXX
 SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ CYCLE_BYTE : 24|8@1+ (1,0) [0|255] "" XXX
"""

  # three banks of eight objects, each object slot is 7 bytes starting at byte 4
  for bank in range(3):
    parts.append(f"\nBO_ {0x180 + bank} OBJECT_GEOMETRY_{bank}: 64 XXX\n" + header)
    for i in range(8):
      parts.append(f""" SG_ DIST_{i} : {39 + 56 * i}|16@0+ (0.005,0) [0|327.675] "m" XXX
 SG_ LAT_{i} : {55 + 56 * i}|12@0- (0.04,0) [-81.92|81.88] "m" XXX
""")

  for bank in range(3):
    parts.append(f"\nBO_ {0x183 + bank} OBJECT_MOTION_{bank}: 64 XXX\n" + header)
    for i in range(8):
      parts.append(f""" SG_ VREL_{i} : {45 + 56 * i}|14@0- (0.025,0) [-204.8|204.775] "m/s" XXX\n""")
    for i in range(8):
      parts.append(f""" SG_ NEW_TRACK_{i} : {71 + 56 * i}|1@0+ (1,0) [0|1] "" XXX
 SG_ TRACK_ENDED_{i} : {76 + 56 * i}|1@0+ (1,0) [0|1] "" XXX
 SG_ TRACK_STATE_{i} : {73 + 56 * i}|2@0+ (1,0) [0|3] "" XXX
""")

  return {"toyota_tss3_radar.dbc": "".join(parts)}
