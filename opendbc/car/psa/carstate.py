from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.values import CAR, DBC, CarControllerParams
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter

class CarState(CarStateBase):
  def update(self, can_parsers) -> structs.CarState:
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_308_T9:
      return self._update_t9(can_parsers[Bus.main])

    cp = can_parsers[Bus.main]
    cp_adas = can_parsers[Bus.adas]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # car speed
    self.parse_wheel_speeds(ret,
      cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'],
      cp.vl['Dyn4_FRE']['P264_VehV_VPsvValWhlFrtR'],
      cp.vl['Dyn4_FRE']['P265_VehV_VPsvValWhlBckL'],
      cp.vl['Dyn4_FRE']['P266_VehV_VPsvValWhlBckR'],
    )
    ret.yawRate = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VITESSE_LACET_BRUTE'] * CV.DEG_TO_RAD
    ret.standstill = bool(cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VEHICLE_STANDSTILL'])

    # gas
    ret.gasPressed = cp.vl['Dyn_CMM']['P002_Com_rAPP'] > 0

    # brake
    ret.brakePressed = bool(cp_cam.vl['Dat_BSI']['P013_MainBrake'])
    ret.parkingBrake = cp.vl['Dyn_EasyMove']['P337_Com_stPrkBrk'] == 1 # 0: disengaged, 1: engaged, 3: brake actuator moving

    # steering wheel
    ret.steeringAngleDeg = cp.vl['STEERING_ALT']['ANGLE'] # EPS
    ret.steeringRateDeg = cp.vl['STEERING_ALT']['RATE'] * (2 * cp.vl['STEERING_ALT']['RATE_SIGN'] - 1) # convert [0,1] to [-1,1] EPS: rot. speed * rot. sign
    ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']
    ret.steeringTorqueEps = cp.vl['IS_DAT_DIRA']['EPS_TORQUE']
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    self.eps_active = cp.vl['IS_DAT_DIRA']['EPS_STATE_LKA'] == 3 # 0: Unauthorized, 1: Authorized, 2: Available, 3: Active, 4: Defect

    # cruise
    ret.cruiseState.speed = cp_adas.vl['HS2_DAT_MDD_CMD_452']['SPEED_SETPOINT'] * CV.KPH_TO_MS # set to 255 when ACC is off, -2 kph offset from dash speed
    ret.cruiseState.enabled = cp_adas.vl['HS2_DAT_MDD_CMD_452']['RVV_ACC_ACTIVATION_REQ'] == 1
    ret.cruiseState.available = cp_adas.vl['HS2_DYN1_MDD_ETAT_2B6']['ACC_STATUS'] > 2
    ret.cruiseState.nonAdaptive = cp_adas.vl['HS2_DAT_MDD_CMD_452']['LONGITUDINAL_REGULATION_TYPE'] != 3 # 0: None, 1: CC, 2: Limiter, 3: ACC
    ret.cruiseState.standstill = bool(cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VEHICLE_STANDSTILL'])
    ret.accFaulted = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['ACC_ETAT_DECEL_OR_ESP_STATUS'] == 3 # 0: Inhibited, 1: Waiting, 2: Active, 3: Fault

    # gear
    if bool(cp_cam.vl['Dat_BSI']['P103_Com_bRevGear']):
      ret.gearShifter = GearShifter.reverse
    else:
      ret.gearShifter = GearShifter.drive

    # blinkers
    blinker = cp_cam.vl['HS2_DAT7_BSI_612']['CDE_CLG_ET_HDC']
    ret.leftBlinker = blinker == 1
    ret.rightBlinker = blinker == 2

    # lock info
    ret.doorOpen = any((cp_cam.vl['Dat_BSI']['DRIVER_DOOR'], cp_cam.vl['Dat_BSI']['PASSENGER_DOOR']))
    ret.seatbeltUnlatched = cp_cam.vl['RESTRAINTS']['DRIVER_SEATBELT'] != 2
    return ret

  def _update_t9(self, cp) -> structs.CarState:
    ret = structs.CarState()
    self.parse_wheel_speeds(ret,
      cp.vl['T9_WHEEL_SPEEDS_30D']['WheelSpeedFrontLeftKph'],
      cp.vl['T9_WHEEL_SPEEDS_30D']['WheelSpeedFrontRightKph'],
      cp.vl['T9_WHEEL_SPEEDS_30D']['WheelSpeedRearLeftKph'],
      cp.vl['T9_WHEEL_SPEEDS_30D']['WheelSpeedRearRightKph'],
    )
    ret.standstill = ret.vEgoRaw < 0.1
    ret.yawRate = cp.vl['T9_BRAKE_DYNAMICS_3CD']['YawRateDegS'] * CV.DEG_TO_RAD
    ret.gasPressed = cp.vl['T9_ENGINE_DYNAMICS_208']['AcceleratorPositionPct'] > 0
    ret.brakePressed = bool(cp.vl['T9_BODY_STATUS_412']['BrakePedalActive'])
    # The 0x412 parking-brake bit never became active in the local corpus.
    ret.parkingBrake = cp.vl['T9_EASY_MOVE_3AD']['ParkingBrakeState'] == 1

    ret.steeringAngleDeg = cp.vl['T9_STEERING_DYNAMICS_305']['SteeringAngleDeg']
    rate = cp.vl['T9_STEERING_DYNAMICS_305']['SteeringRateMagnitudeDegS']
    rate_sign = cp.vl['T9_STEERING_DYNAMICS_305']['SteeringRateSign']
    ret.steeringRateDeg = rate * (1 if rate_sign == 0 else -1)
    ret.steeringTorque = cp.vl['T9_STEERING_TORQUE_2F5']['DriverTorqueRaw']
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.T9_STEER_DRIVER_THRESHOLD_RAW, 5)

    # 0x208 carries the persistent RVV state; 0x452 carries a request, not a latch.
    cruise_mode = cp.vl['T9_CRUISE_SETPOINT_50E']['CruiseMode']
    ret.cruiseState.available = cruise_mode == 1
    ret.cruiseState.enabled = cruise_mode == 1 and cp.vl['T9_ENGINE_DYNAMICS_208']['CruiseStateCandidate'] == 2
    setpoint = cp.vl['T9_CRUISE_SETPOINT_50E']['CruiseSetpointKph']
    ret.cruiseState.speed = setpoint * CV.KPH_TO_MS if cruise_mode == 1 and setpoint < 255 else 0.
    # Only the conventional-cruise reference vehicle is covered.
    ret.cruiseState.nonAdaptive = True

    reverse = bool(cp.vl['T9_BODY_STATUS_412']['ReverseGearActive'])
    # The candidate gear field in 0x348 stays zero even in the moving capture.
    # Only the independent reverse indication is used until gear is validated.
    ret.gearShifter = GearShifter.reverse if reverse else GearShifter.unknown

    blinker = cp.vl['T9_DRIVER_CRUISE_COMMAND_452']['TurnSignalStatus']
    ret.leftBlinker = blinker in (2, 3)
    ret.rightBlinker = blinker in (1, 3)
    ret.doorOpen = any(cp.vl['T9_BODY_STATUS_412'][signal] for signal in (
      'DriverDoorOpen', 'PassengerDoorOpen', 'RearLeftDoorOpen', 'RearRightDoorOpen',
    ))
    ret.seatbeltUnlatched = cp.vl['T9_RESTRAINTS_572']['DriverSeatbeltState'] != 2
    return ret

  @staticmethod
  def get_can_parsers(CP):
    if CP.carFingerprint == CAR.PSA_PEUGEOT_308_T9:
      # Only the observed live CAN stream is mapped to logical bus 0.
      # A split camera/powertrain harness topology has not been validated.
      messages = [
        ('T9_ENGINE_DYNAMICS_208', 100),
        ('T9_STEERING_TORQUE_2F5', 100),
        ('T9_STEERING_DYNAMICS_305', 100),
        ('T9_WHEEL_SPEEDS_30D', 50),
        ('T9_EASY_MOVE_3AD', 50),
        ('T9_BRAKE_DYNAMICS_3CD', 100),
        ('T9_BODY_STATUS_412', 20),
        ('T9_DRIVER_CRUISE_COMMAND_452', 20),
        ('T9_CRUISE_SETPOINT_50E', 10),
        ('T9_RESTRAINTS_572', 10),
      ]
      return {Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], messages, 0)}

    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
