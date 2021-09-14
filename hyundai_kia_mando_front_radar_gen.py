#!/usr/bin/env python3
print("""
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

print("""
BO_ 513 R_0x201: 8 XXX

BO_ 768 R_0x300: 8 XXX
""")

for s, l in [(0x400, 24), (0x418, 16)]:
  for a in range(0x400, 0x400+40):
    print(f"""
BO_ {a} R_{hex(a)}: 8 XXX
""")

print("""
BO_ 1248 R_0x4E0: 8 XXX
BO_ 1249 R_0x4E1: 8 XXX
BO_ 1250 R_0x4E2: 8 XXX
BO_ 1251 R_0x4E3: 8 XXX
BO_ 1252 R_0x4E4: 8 XXX
BO_ 1253 R_0x4E5: 8 XXX
BO_ 1254 R_0x4E6: 8 XXX
BO_ 1255 R_0x4E7: 8 XXX

BO_ 1530 R_0x5FA: 8 XXX
BO_ 1531 R_0x5FB: 8 XXX

BO_ 1533 R_0x5FD: 8 XXX
BO_ 1534 R_0x5FE: 8 XXX
BO_ 1535 R_0x5FF: 8 XXX

""")

# note: 0x501/0x502 seem to be special in 0x5XX range
for s, l in [(0x500, 32), (0x600, 70), (0x646, 40)]:
  for a in range(s, s+l):
    print(f"""
BO_ {a} R_{hex(a)}: 8 XXX
 SG_ NEW_SIGNAL_1 : 7|8@0- (1,0) [-128|127] "" XXX
 SG_ NEW_SIGNAL_2 : 12|10@0- (0.2,0) [-102.4|102.2] "" XXX
 SG_ NEW_SIGNAL_3 : 15|3@0+ (1,0) [0|7] "" XXX
 SG_ NEW_SIGNAL_4 : 18|11@0+ (0.1,0) [0|204.7] "" XXX
 SG_ NEW_SIGNAL_5 : 33|10@0- (0.2,0) [-102.4|102.2] "" XXX
 SG_ NEW_SIGNAL_6 : 37|4@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_7 : 38|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_8 : 39|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_9 : 53|14@0- (0.04,0) [-327.68|327.64] "" XXX
 SG_ NEW_SIGNAL_10 : 55|2@0+ (1,0) [0|3] "" XXX
""")

for s, l in [(0x680, 16), (0x690, 16)]:
  for a in range(s, s+l):
    print(f"""
BO_ {a} R_{hex(a)}: 8 XXX
""")

print("""
BO_ 1696 R_0x6A0: 8 XXX
BO_ 1697 R_0x6A1: 8 XXX
BO_ 1698 R_0x6A2: 8 XXX
BO_ 1699 R_0x6A3: 8 XXX
BO_ 1700 R_0x6A4: 8 XXX
""")
