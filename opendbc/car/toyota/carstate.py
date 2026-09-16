import copy

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarStateBase
from opendbc.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, EPS_SCALE, \
                                                  TSS3_PT_BUS, TSS3_STEER_THRESHOLD

ButtonType = structs.CarState.ButtonEvent.Type
SteerControlType = structs.CarParams.SteerControlType

# These steering fault definitions seem to be common across LKA (torque) and LTA (angle):
# - high steer rate fault: goes to 21 or 25 for 1 frame, then 9 for 2 seconds
# - lka/lta msg drop out: goes to 9 then 11 for a combined total of 2 seconds, then 3.
#     if using the other control command, goes directly to 3 after 1.5 seconds
# - initializing: LTA can report 0 as long as STEER_TORQUE_SENSOR->STEER_ANGLE_INITIALIZING is 1,
#     and is a catch-all for LKA
TEMP_STEER_FAULTS = (0, 9, 11, 21, 25)
# - lka/lta msg drop out: 3 (recoverable)
# - prolonged high driver torque: 17 (permanent)
PERM_STEER_FAULTS = (3, 17)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    if CP.flags & ToyotaFlags.SECOC.value and not CP.flags & ToyotaFlags.CAN_FD.value:
      self.shifter_values = can_define.dv["GEAR_PACKET_HYBRID"]["GEAR"]
    else:
      self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)

    self.lkas_button = 0
    self.distance_button = 0

    self.pcm_follow_distance = 0

    self.acc_type = 1
    self.lkas_hud = {}
    self.gvc = 0.0
    self.secoc_synchronization = None

    # TSS 3.0 (modify-and-forward): the camera's live 0x160 frame, captured on the
    # cam bus, is the template the carcontroller edits (accel bytes 4-5 and steer
    # bytes 22-23). tss3_camera_accel is what the camera itself requested;
    # tss3_stock_lon_active gates override on the stock ACC actually controlling.
    self.tss3_accel_template = None
    self.tss3_camera_accel = 0.0
    self.tss3_stock_lon_active = False
    self.tss3_steer_template = None

  def update(self, can_parsers) -> structs.CarState:
    if self.CP.flags & ToyotaFlags.CAN_FD.value:
      return self.update_tss3(can_parsers)

    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()
    cp_acc = cp_cam if (self.CP.flags & ToyotaFlags.TSS2) and not (self.CP.flags & ToyotaFlags.RADAR_ACC) else cp

    if not self.CP.flags & ToyotaFlags.SECOC.value:
      self.gvc = cp.vl["VSC1S07"]["GVC"]

    ret.doorOpen = any([cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FR"],
                        cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0
    ret.parkingBrake = cp.vl["BODY_CONTROL_STATE"]["PARKING_BRAKE"] == 1

    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    ret.brakeHoldActive = cp.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1

    if self.CP.flags & ToyotaFlags.SECOC.value:
      self.secoc_synchronization = copy.copy(cp.vl["SECOC_SYNCHRONIZATION"])
      ret.gasPressed = cp.vl["GAS_PEDAL"]["GAS_PEDAL_USER"] > 0
      can_gear = int(cp.vl["GEAR_PACKET_HYBRID"]["GEAR"])
    else:
      ret.gasPressed = cp.vl["PCM_CRUISE"]["GAS_RELEASED"] == 0
      can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])
      if not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
        ret.stockAeb = bool(cp_acc.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_acc.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoCluster = ret.vEgo * 1.015  # minimum of all the cars

    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    ret.vehicleSensorsInvalid = any(cp.vl["WHEEL_SPEEDS"][f"WHEEL_SPEED_{whl}_FAULT"]
                                    for whl in ("FL", "FR", "RL", "RR"))

    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
    torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]

    # On some cars, the angle measurement is non-zero while initializing
    if abs(torque_sensor_angle_deg) > 1e-3 and not bool(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE_INITIALIZING"]):
      self.accurate_steer_angle_seen = True

    if self.accurate_steer_angle_seen:
      # Offset seems to be invalid for large steering angles and high angle rates
      if abs(ret.steeringAngleDeg) < 90 and abs(ret.steeringRateDeg) < 100 and cp.can_valid:
        self.angle_offset.update(torque_sensor_angle_deg - ret.steeringAngleDeg)

      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = torque_sensor_angle_deg - self.angle_offset.x

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # Check EPS LKA/LTA fault status
    ret.steerFaultTemporary = cp.vl["EPS_STATUS"]["LKA_STATE"] in TEMP_STEER_FAULTS
    ret.steerFaultPermanent = cp.vl["EPS_STATUS"]["LKA_STATE"] in PERM_STEER_FAULTS

    if self.CP.steerControlType == SteerControlType.angle:
      ret.steerFaultTemporary = ret.steerFaultTemporary or cp.vl["EPS_STATUS"]["LTA_STATE"] in TEMP_STEER_FAULTS
      ret.steerFaultPermanent = ret.steerFaultPermanent or cp.vl["EPS_STATUS"]["LTA_STATE"] in PERM_STEER_FAULTS

      # Lane Tracing Assist control is unavailable (EPS_STATUS->LTA_STATE=0) until
      # the more accurate angle sensor signal is initialized
      if not self.accurate_steer_angle_seen:
        ret.vehicleSensorsInvalid = True

    if self.CP.flags & ToyotaFlags.UNSUPPORTED_DSU:
      # TODO: find the bit likely in DSU_CRUISE that describes an ACC fault. one may also exist in CLUTCH
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_ALT"]["UI_SET_SPEED"]
    else:
      ret.accFaulted = cp.vl["PCM_CRUISE_2"]["ACC_FAULTED"] != 0
      ret.carFaultedNonCritical = cp.vl["PCM_CRUISE_SM"]["TEMP_ACC_FAULTED"] != 0
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_SM"]["UI_SET_SPEED"]

    # UI_SET_SPEED is always non-zero when main is on, hide until first enable
    is_metric = cp.vl["BODY_CONTROL_STATE_2"]["UNITS"] in (1, 2)
    if ret.cruiseState.speed != 0:
      conversion_factor = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
      ret.cruiseState.speedCluster = cluster_set_speed * conversion_factor

    if self.CP.flags & ToyotaFlags.TSS2 and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      self.acc_type = cp_acc.vl["ACC_CONTROL"]["ACC_TYPE"]
      ret.stockFcw = bool(cp_acc.vl["PCS_HUD"]["FCW"])

    # some TSS2 cars have low speed lockout permanently set, so ignore on those cars
    # these cars are identified by an ACC_TYPE value of 2.
    # TODO: it is possible to avoid the lockout and gain stop and go if you
    # send your own ACC_CONTROL msg on startup with ACC_TYPE set to 1
    if (not (self.CP.flags & ToyotaFlags.TSS2) and not (self.CP.flags & ToyotaFlags.UNSUPPORTED_DSU)) or \
       (self.CP.flags & ToyotaFlags.TSS2 and self.acc_type == 1):
      if self.CP.openpilotLongitudinalControl:
        ret.accFaulted = ret.accFaulted or cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2

    pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]
    ret.cruiseState.standstill = pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = pcm_acc_status in (1, 2, 3, 4, 5, 6)

    ret.genericToggle = bool(cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"])
    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0

    if self.CP.flags & ToyotaFlags.HAS_BSM:
      ret.leftBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)

    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      self.lkas_hud = copy.copy(cp_cam.vl["LKAS_HUD"])

    if not (self.CP.flags & ToyotaFlags.UNSUPPORTED_DSU):
      self.pcm_follow_distance = cp.vl["PCM_CRUISE_2"]["PCM_FOLLOW_DISTANCE"]

    buttonEvents = []
    if self.CP.flags & ToyotaFlags.TSS2:
      # lkas button is wired to the camera
      prev_lkas_button = self.lkas_button
      self.lkas_button = cp_cam.vl["LKAS_HUD"]["LDA_ON_MESSAGE"]

      # Cycles between 1 and 2 when pressing the button, then rests back at 0 after ~3s
      if self.lkas_button != 0 and self.lkas_button != prev_lkas_button:
        buttonEvents.extend(create_button_events(1, 0, {1: ButtonType.lkas}) +
                            create_button_events(0, 1, {1: ButtonType.lkas}))

      if not (self.CP.flags & (ToyotaFlags.RADAR_ACC | ToyotaFlags.SECOC)):
        # distance button is wired to the ACC module (camera or radar)
        prev_distance_button = self.distance_button
        self.distance_button = cp_acc.vl["ACC_CONTROL"]["DISTANCE"]

        buttonEvents += create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    ret.buttonEvents = buttonEvents
    return ret

  def update_tss3(self, can_parsers) -> structs.CarState:
    """TSS 3.0 (CAN FD + SecOC) -- Phase 1, READ ONLY.

    Every signal here is decoded from passive rlogs and has never been validated
    on the car. cruiseState is hard-stubbed off so openpilot cannot believe it is
    allowed to engage; there is no carcontroller path for this platform.
    """
    cp = can_parsers[Bus.pt]

    ret = structs.CarState()

    # parse_wheel_speeds sets vEgoRaw/vEgo/aEgo. It does not populate
    # ret.wheelSpeeds.* -- nothing in the Toyota path does. Default unit is
    # CV.KPH_TO_MS, which matches this DBC's km/h.
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    # Positive = left (verified in logs). No STEER_FRACTION/STEER_RATE decoded,
    # so the classic angle-offset cross-check does not apply here.
    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_ACC_STATUS"]["STEER_ANGLE"]
    ret.steeringRateDeg = 0.
    ret.yawRate = cp.vl["KINEMATICS"]["YAW_RATE"]

    # 0xDA STEER_TORQUE_SENSOR carries four int16s. Identified on-car: TORQUE_3 is the
    # DRIVER torque (bidirectional, spikes at takeovers/lane-change nudges, low correlation
    # with steering angle), TORQUE_1 tracks steering effort (EPS/column). steeringPressed
    # enables driver override so retaking the wheel pauses openpilot lateral rather than
    # fighting it. TSS3_STEER_THRESHOLD calibrated on-car.
    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["TORQUE_3"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["TORQUE_1"]
    ret.steeringPressed = abs(ret.steeringTorque) > TSS3_STEER_THRESHOLD

    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    # 0x116 byte 1, driver gas pedal. Rest is a true 0 (100% zero while
    # braking), so != 0 matches the panda's own gas check exactly.
    ret.gasPressed = cp.vl["GAS_PEDAL"]["GAS_PEDAL_USER"] != 0

    ret.gearShifter = self.parse_gear_shifter(
      self.shifter_values.get(int(cp.vl["GEAR_PACKET"]["GEAR"]), None))

    # ACC state from 0x8A, 40 Hz. DECODED FROM A REAL DRIVE 2026-09-09:
    #   ACC_STATE  byte 7  : 0x12 = on but not engaged, 0x47 = engaged
    #   ACC_ENGAGED byte 22 mask 0x10 : the clean engaged bit
    # Both are INDEPENDENT of 0x13C and lead it by ~100ms, so they report the
    # DRIVER'S intent, not the command. That is what makes LIVE possible: when
    # openpilot transmits 0x13C these still report engage and cancel correctly.
    # ACC_STATE is BINARY on this car: 0x12 (not engaged) or 0x47 (engaged),
    # and nothing else across 7232 frames covering three engagements. There is
    # no "powered on but not engaged" standby state, because the ACC main
    # button engages directly at the current speed -- confirmed by the owner,
    # and consistent with every transition being a single 0x12 -> 0x47 step
    # with no intermediate value.
    #
    # So `available` is not a state this car reports: the system is offerable
    # whenever it is responding at all. Guarding on != 0 keeps a fault (no
    # frames -> 0) from reading as available.
    acc_state = int(cp.vl["STEER_ANGLE_ACC_STATUS"]["ACC_STATE"])
    ret.cruiseState.available = acc_state != 0
    ret.cruiseState.enabled = bool(cp.vl["STEER_ANGLE_ACC_STATUS"]["ACC_ENGAGED"])

    # 0x251 byte 2, increments exactly one per +/- press. CONFIRMED mph against
    # the dash on 2026-09-09, factor 1.0 (the raw byte IS the displayed number).
    ret.cruiseState.speed = cp.vl["ACC_HUD"]["SET_SPEED"] * CV.MPH_TO_MS
    # 0x8A byte 7 mask 0x20: engaged and holding at a stop behind a lead car.
    # Captured 2026-09-09 (492 frames at 0.0 kph). openpilot uses this to raise
    # resumeRequired instead of assuming it may just drive off.
    ret.cruiseState.standstill = bool(cp.vl["STEER_ANGLE_ACC_STATUS"]["ACC_STANDSTILL"])

    # Turn signals: BODY_CONTROL_STATE (0x614), byte3 bits 4-5 (TURN_SIGNALS) == 1 left,
    # 2 right, 3 off (standard Toyota code). Confirmed on-car.
    ret.leftBlinker = cp.vl["BODY_CONTROL_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BODY_CONTROL_STATE"]["TURN_SIGNALS"] == 2

    # LTA hands-off: LKAS_HUD (0x412) escalates byte1=0x0c (hands-on nag) then byte2 bit6
    # = LTA_DISABLE when the driver keeps hands off; the EPS then stops applying the 0x160
    # steer. Report the disable as a temporary steer fault so openpilot alerts and hands
    # back instead of silently commanding a dead EPS. Confirmed on-car (only ever on true
    # hands-off, never in normal driving).
    ret.steerFaultTemporary = bool(cp.vl["LKAS_HUD"]["LTA_DISABLE"])
    ret.steerFaultPermanent = False

    # Not decoded on this platform -- inert rather than guessed.
    ret.doorOpen = False
    ret.seatbeltUnlatched = False
    ret.buttonEvents = []

    # Kept for a future Phase 2. Note opendbc's add_mac() only handles 8-byte
    # SecOC frames; every signed message here except 0x0F is 32-byte CAN FD.
    self.secoc_synchronization = copy.copy(cp.vl["SECOC_SYNCHRONIZATION"])

    # ---- TSS 3.0 longitudinal template capture (read-only) ----------------
    # Reconstruct the camera's exact 0x160 frame from the cam-bus parser so the
    # carcontroller can modify-and-forward it. 0x160 is E2E-protected (keyless
    # CRC + counter), so this is all openpilot needs to regenerate valid frames.
    cam = can_parsers[Bus.cam]
    adas = cam.vl["ADAS_ACC_REQUEST"]
    self.tss3_accel_template = bytes(int(adas[f"BYTE{k:02d}"]) & 0xFF for k in range(32))
    self.tss3_camera_accel = float(adas["ACCEL_REQ"])
    # Lateral rides in 0x160 too (bytes 22-23), so the 0x160 template above is all
    # openpilot needs for BOTH accel and steer. The gateway's 0x1A0 is NOT read:
    # it is gateway-native on bus0 and only reaches bus2 as low-rate forwarded
    # copies once the relay closes, so requiring it at 50 Hz tripped canError.
    # 0x13C on the powertrain bus reports whether the STOCK ACC is actively
    # controlling. v1 only overrides accel while this is true -- the driver
    # engages with the stalk exactly as stock; openpilot rides that engagement.
    self.tss3_stock_lon_active = bool(cp.vl["ACC_CONTROL"]["LON_ACTIVE"])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    if CP.flags & ToyotaFlags.CAN_FD.value:
      # float('nan') sets ignore_alive=True: no liveness check. Used for every
      # message whose rate is not stated in the port doc -- guessing a rate
      # would make CANParser mark the message not-valid and block engagement.
      # Replace each nan with the measured rate as it is confirmed from an rlog.
      tss3_messages = [
        ("WHEEL_SPEEDS", float('nan')),
        ("STEER_ANGLE_ACC_STATUS", 40),     # measured
        ("STEER_ANGLE_SENSOR", float('nan')),
        ("KINEMATICS", float('nan')),
        ("STEER_TORQUE_SENSOR", 42),        # measured
        ("BRAKE_MODULE", float('nan')),
        ("GEAR_PACKET", float('nan')),
        ("SECOC_SYNCHRONIZATION", 10),      # measured
        ("ACC_CONTROL", 20),                # measured: 20 Hz, plaintext, bus 0
        ("GAS_PEDAL", float('nan')),        # 0x116, driver gas pedal on bus 1
        ("ACC_HUD", float('nan')),          # 0x251, ~1.3 Hz cluster msg (set speed)
        ("BODY_CONTROL_STATE", float('nan')),  # 0x614, turn signals (bus 1)
        ("LKAS_HUD", float('nan')),            # 0x412, LTA hands-on nag / disable (bus 1)
      ]
      # With STOCK harness wiring the powertrain lands on TSS3_PT_BUS (bus 1,
      # unrelayed) and the relayed pair (bus 0 <-> bus 2) carries the ADAS CAN FD
      # bus instead. None of the messages in this DBC exist there, so the cam
      # parser is empty -- it exists only to satisfy the expected shape.
      # The camera's 0x160 ACC request rides the ADAS bus (relayed pair). Read
      # it on bus 2 (the camera side) so the template is the genuine camera frame
      # even once the relay opens and bus 0 carries openpilot's replacement.
      tss3_cam_messages = [
        ("ADAS_ACC_REQUEST", 40),   # 0x160: carries BOTH accel and the steer request
      ]
      return {
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], tss3_messages, TSS3_PT_BUS),
        Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], tss3_cam_messages, 2),
      }

    pt_messages = [
      ("BLINKERS_STATE", float('nan')),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
