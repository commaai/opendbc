#!/usr/bin/env python3
import os

if __name__ == "__main__":
  dbc_name = os.path.basename(__file__).replace(".py", ".dbc")
  hyundai_path = os.path.dirname(os.path.realpath(__file__))
  with open(os.path.join(hyundai_path, dbc_name), "w", encoding='utf-8') as f:
    f.write("""
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
    """)

    for a in range(0x3A5, 0x3A5 + 32):
        f.write(f"""
BO_ {a} RADAR_TRACK_{a:x}: 24 RADAR
 SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" XXX
 SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_1 : 25|2@0+ (1,0) [0|3] "" XXX
 SG_ NEW_SIGNAL_3 : 28|2@0+ (1,0) [0|3] "" XXX
 SG_ COUNTER_3 : 31|2@0+ (1,0) [0|3] "" XXX
 SG_ NEW_SIGNAL_2 : 38|7@0- (1,0) [0|127] "" XXX
 SG_ COUNTER_256 : 47|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_6 : 51|4@0+ (1,0) [0|15] "" XXX
 SG_ STATE : 54|3@0+ (1,0) [0|7] "" XXX
 SG_ NEW_SIGNAL_8 : 62|7@0- (1,0) [0|127] "" XXX
 SG_ LONG_DIST : 63|12@1+ (0.05,0) [0|8191] "m" XXX
 SG_ LAT_DIST : 76|12@1- (0.05,0) [0|127] "" XXX
 SG_ REL_SPEED : 88|14@1- (0.01,0) [0|16383] "" XXX
 SG_ NEW_SIGNAL_4 : 103|2@0+ (1,0) [0|3] "" XXX
 SG_ LAT_DIST_ACCEL : 104|13@1- (1,0) [0|8191] "" XXX
 SG_ REL_ACCEL : 118|10@1- (0.02,0) [0|1023] "" XXX
 SG_ NEW_SIGNAL_5 : 133|4@0+ (1,0) [0|15] "" XXX
    """)
