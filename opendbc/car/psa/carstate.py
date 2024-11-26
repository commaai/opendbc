from cereal import car
from opendbc.can.parser import CANParser
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.psa.psacan import CanBus
from openpilot.selfdrive.car.psa.values import DBC, CarControllerParams
from openpilot.selfdrive.car.interfaces import CarStateBase

GearShifter = car.CarState.GearShifter
TransmissionType = car.CarParams.TransmissionType


class CarState(CarStateBase):
  def update(self, cp, cp_adas, cp_cam):
    ret = car.CarState.new_message()

    # car speed
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'],
      cp.vl['Dyn4_FRE']['P264_VehV_VPsvValWhlFrtR'],
      cp.vl['Dyn4_FRE']['P265_VehV_VPsvValWhlBckL'],
      cp.vl['Dyn4_FRE']['P266_VehV_VPsvValWhlBckR'],
    )
    ret.vEgoRaw = cp_adas.vl['HS2_DYN_ABR_38D']['VITESSE_VEHICULE_ROUES'] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.yawRate = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VITESSE_LACET_BRUTE'] * CV.DEG_TO_RAD
    ret.standstill = False  # TODO

    # gas
    ret.gas = cp_adas.vl['HS2_BGE_DYN5_CMM_228']['EFCMNT_PDLE_ACCEL'] / 99.5
    ret.gasPressed = ret.gas > 0  # TODO

    # brake
    # ret.brake = cp.vl['Dyn2_FRE']['BRAKE_PRESSURE'] / 1500.  # FIXME
    ret.brakePressed = bool(cp.vl['Dat_BSI']['P013_MainBrake'])
    ret.parkingBrake = bool(cp.vl['Dat_BSI']['PARKING_BRAKE'])  # TODO: check e-brake

    # steering wheel
    ret.steeringAngleDeg = cp.vl['STEERING_ALT']['ANGLE']
    ret.steeringRateDeg = cp.vl['STEERING_ALT']['RATE'] * cp.vl['STEERING_ALT']['RATE_SIGN']  # TODO: check units
    ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']  # TODO: check units
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)  # TODO: adjust threshold
    ret.steerFaultTemporary = False  # TODO
    ret.steerFaultPermanent = False  # TODO
    ret.espDisabled = False  # TODO

    # cruise
    # note: this is just for CC car not ACC right now
    ret.cruiseState.speed = cp_adas.vl['HS2_DAT_MDD_CMD_452']['CONS_LIM_VITESSE_VEH'] * CV.KPH_TO_MS
    ret.cruiseState.enabled = cp_adas.vl['HS2_DAT_MDD_CMD_452']['DDE_ACTIVATION_RVV_ACC'] == 1
    ret.cruiseState.available = True  # TODO
    ret.cruiseState.nonAdaptive = False  # cp_adas.vl['HS2_DAT_MDD_CMD_452']['COCKPIT_GO_ACC_REQUEST'] == 0
    ret.cruiseState.standstill = False  # TODO
    ret.accFaulted = False

    # gear
    # TODO: automatic transmission gear
    if self.CP.transmissionType == TransmissionType.manual:
      ret.clutchPressed = bool(cp.vl['Dyn2_CMM']['P091_ConvCD_stDebVal'])
      if bool(cp.vl['Dat_BSI']['P103_Com_bRevGear']):
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # TODO: safety
    ret.stockFcw = False
    ret.stockAeb = False

    # button presses
    blinker = cp_adas.vl['HS2_DAT7_BSI_612']['CDE_CLG_ET_HDC']
    ret.leftBlinker = blinker == 1
    ret.rightBlinker = blinker == 2

    # lock info
    ret.doorOpen = any([cp.vl['Dat_BSI']['DRIVER_DOOR'], cp.vl['Dat_BSI']['PASSENGER_DOOR']])
    ret.seatbeltUnlatched = cp.vl['RESTRAINTS']['DRIVER_SEATBELT'] != 2  # TODO: check LHD

    return ret

  @staticmethod
  def get_can_parser(CP):
    messages = [
      ('Dyn4_FRE', 50),
      ('Dat_BSI', 20),
      ('STEERING_ALT', 100),
      ('STEERING', 100),
      ('Dyn2_CMM', 50),
      ('RESTRAINTS', 10),
    ]
    return CANParser(DBC[CP.carFingerprint]['pt'], messages, CanBus(CP).main)

  @staticmethod
  def get_adas_can_parser(CP):
    messages = [
      ('HS2_DYN_ABR_38D', 25),
      ('HS2_DYN_UCF_MDD_32D', 50),
      ('HS2_BGE_DYN5_CMM_228', 100),
      ('HS2_DAT_MDD_CMD_452', 20),
      ('HS2_DAT7_BSI_612', 10),
    ]
    return CANParser(DBC[CP.carFingerprint]['body'], messages, CanBus(CP).adas)
