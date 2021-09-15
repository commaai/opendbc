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

# camera messages ?
# TODO: not all the same
for s, l in [(0x202, 8), (0x20A, 4), (0x238, 30), (0x25A, 5), (0x266, 10)]:
  for a in range(s, s+l):
    print(f"""
BO_ {a} V_{hex(a)}: 8 XXX
""")

# radar messages
print("""
BO_ 513 R_0x201: 8 XXX
 SG_ NEW_SIGNAL_1 : 55|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_2 : 63|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_3 : 0|2@1+ (1,0) [0|3] "" XXX
 SG_ NEW_SIGNAL_4 : 7|6@0+ (1,0) [0|31] "" XXX

BO_ 768 R_0x300: 8 XXX
""")

for s, l in [(0x400, 24), (0x418, 16)]:
  for a in range(s, s+l):
    print(f"""
BO_ {a} R_{hex(a)}: 8 XXX
""")

print("""
BO_ 1248 R_0x4E0: 8 XXX
 SG_ NEW_SIGNAL_1 : 31|16@0+ (1,0) [0|65535] "" XXX
 SG_ NEW_SIGNAL_4 : 47|12@0- (0.0625,0) [0|7] "" XXX
 SG_ NEW_SIGNAL_3 : 50|11@0+ (0.2,0) [0|511] "" XXX
 SG_ NEW_SIGNAL_5 : 7|2@0+ (1,0) [0|3] "" XXX
 SG_ NEW_SIGNAL_2 : 13|14@0+ (0.01,0) [0|16383] "" XXX

BO_ 1249 R_0x4E1: 8 XXX
 SG_ NEW_SIGNAL_1 : 15|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_2 : 31|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_3 : 10|2@1+ (1,0) [0|3] "" XXX
 SG_ NEW_SIGNAL_4 : 13|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_5 : 12|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_6 : 9|10@0+ (1,0) [0|7] "" XXX
 SG_ NEW_SIGNAL_7 : 55|10@0+ (1,0) [0|1023] "" XXX

BO_ 1250 R_0x4E2: 8 XXX
 SG_ NEW_SIGNAL_1 : 7|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_2 : 15|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_3 : 23|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_4 : 31|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_5 : 39|8@0+ (1,0) [0|255] "" XXX

BO_ 1251 R_0x4E3: 8 XXX
 SG_ NEW_SIGNAL_1 : 15|8@0+ (1,0) [0|65535] "" XXX
 SG_ NEW_SIGNAL_2 : 23|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_3 : 31|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_4 : 63|8@0+ (1,0) [0|255] "" XXX

BO_ 1252 R_0x4E4: 8 XXX
 SG_ NEW_SIGNAL_1 : 7|16@0+ (1,0) [0|65535] "" XXX
 SG_ NEW_SIGNAL_2 : 23|16@0+ (1,0) [0|65535] "" XXX
 SG_ NEW_SIGNAL_3 : 39|16@0+ (1,0) [0|65535] "" XXX
 SG_ NEW_SIGNAL_4 : 55|16@0+ (1,0) [0|65535] "" XXX

BO_ 1253 R_0x4E5: 8 XXX
 SG_ NEW_SIGNAL_1 : 0|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_2 : 1|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_3 : 2|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_4 : 3|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_5 : 4|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_6 : 5|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_7 : 6|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_8 : 7|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_9 : 8|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_10 : 9|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_11 : 10|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_12 : 11|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_13 : 12|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_14 : 13|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_15 : 14|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_16 : 15|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_17 : 16|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_18 : 17|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_19 : 18|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_20 : 19|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_21 : 20|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_22 : 21|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_23 : 22|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_24 : 23|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_25 : 24|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_26 : 25|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_27 : 26|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_28 : 27|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_29 : 28|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_30 : 29|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_31 : 30|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_32 : 31|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_33 : 32|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_34 : 33|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_35 : 34|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_36 : 35|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_37 : 36|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_38 : 37|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_39 : 38|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_40 : 39|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_41 : 40|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_42 : 41|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_43 : 42|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_44 : 43|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_45 : 44|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_46 : 45|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_47 : 46|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_48 : 47|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_49 : 48|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_50 : 49|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_51 : 50|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_52 : 51|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_53 : 52|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_54 : 53|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_55 : 54|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_56 : 55|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_57 : 56|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_58 : 57|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_59 : 58|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_60 : 59|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_61 : 60|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_62 : 61|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_63 : 62|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_64 : 63|1@0+ (1,0) [0|1] "" XXX

BO_ 1254 R_0x4E6: 8 XXX
 SG_ NEW_SIGNAL_1 : 7|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_2 : 15|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_3 : 23|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_4 : 31|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_5 : 39|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_6 : 47|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_7 : 55|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_8 : 63|8@0+ (1,0) [0|255] "" XXX

BO_ 1255 R_0x4E7: 8 XXX
 SG_ NEW_SIGNAL_1 : 0|10@1+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_2 : 10|10@1+ (1,0) [0|63] "" XXX
 SG_ NEW_SIGNAL_7 : 20|2@1+ (1,0) [0|3] "" XXX
 SG_ NEW_SIGNAL_3 : 22|8@1+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_5 : 32|10@1+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_4 : 42|10@1+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_6 : 52|1@1+ (1,0) [0|15] "" XXX

BO_ 1530 R_0x5FA: 8 XXX
BO_ 1531 R_0x5FB: 8 XXX

BO_ 1533 R_0x5FD: 8 XXX
BO_ 1534 R_0x5FE: 8 XXX

BO_ 1535 R_0x5FF: 8 XXX
 SG_ NEW_SIGNAL_1 : 0|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_2 : 1|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_3 : 2|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_4 : 3|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_5 : 4|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_6 : 5|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_7 : 6|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_8 : 7|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_9 : 8|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_10 : 9|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_11 : 10|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_12 : 11|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_13 : 12|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_14 : 13|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_15 : 14|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_16 : 15|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_17 : 16|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_18 : 17|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_19 : 18|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_20 : 19|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_21 : 20|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_22 : 21|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_23 : 22|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_24 : 23|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_25 : 24|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_26 : 25|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_27 : 26|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_28 : 27|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_29 : 28|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_30 : 29|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_31 : 30|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_32 : 31|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_33 : 32|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_34 : 33|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_35 : 34|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_36 : 35|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_37 : 36|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_38 : 37|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_39 : 38|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_40 : 39|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_41 : 40|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_42 : 41|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_43 : 42|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_44 : 43|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_45 : 44|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_46 : 45|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_47 : 46|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_48 : 47|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_49 : 48|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_50 : 49|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_51 : 50|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_52 : 51|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_53 : 52|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_54 : 53|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_55 : 54|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_56 : 55|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_57 : 56|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_58 : 57|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_59 : 58|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_60 : 59|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_61 : 60|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_62 : 61|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_63 : 62|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_64 : 63|1@0+ (1,0) [0|1] "" XXX
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
BO_ 1696 R_0x5ED: 8 XXX
BO_ 1697 R_0x5EE: 8 XXX
BO_ 1698 R_0x5EF: 8 XXX

BO_ 1696 R_0x6A0: 8 XXX
BO_ 1697 R_0x6A1: 8 XXX
BO_ 1698 R_0x6A2: 8 XXX
BO_ 1699 R_0x6A3: 8 XXX
BO_ 1700 R_0x6A4: 8 XXX
""")
