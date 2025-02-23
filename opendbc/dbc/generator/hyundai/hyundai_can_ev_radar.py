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

    # note: 0x501/0x502 seem to be special in 0x5XX range
    for a in range(0x602, 0x602 + 16):
        f.write(f"""
BO_ {a} RADAR_TRACK_{a:x}: 8 RADAR
 SG_ DISTANCE : 0|10@1+ (0.1,0) [-128|127] "" XXX
 SG_ LATERAL : 10|10@1- (1,0) [0|511] "" XXX
 SG_ SPEED : 21|9@1- (1,0) [0|63] "" XXX
 SG_ NEW_SIGNAL_3 : 31|10@1+ (1,0) [0|511] "" XXX
 SG_ NEW_SIGNAL_4 : 41|10@1- (1,0) [0|1023] "" XXX
 SG_ NEW_SIGNAL_5 : 52|9@1- (1,0) [0|15] "" XXX
 SG_ COUNTER : 62|2@1+ (1,0) [0|3] "" XXX
    """)
