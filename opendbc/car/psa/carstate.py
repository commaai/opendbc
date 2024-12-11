from opendbc.car import structs, Bus
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.psacan import CanBus
from opendbc.car.psa.values import DBC, CarControllerParams
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType

# TODO: merge DBC files or separate into CAN0/2 (HS1) and CAN1 (HS2)
class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    # cp_adas = can_parsers[Bus.adas] # TODO check if needed
    ret = structs.CarState()

    # car speed
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'], # HS1
      cp.vl['Dyn4_FRE']['P264_VehV_VPsvValWhlFrtR'],
      cp.vl['Dyn4_FRE']['P265_VehV_VPsvValWhlBckL'],
      cp.vl['Dyn4_FRE']['P266_VehV_VPsvValWhlBckR'],
    )
    ret.vEgoRaw = cp.vl['HS2_DYN_ABR_38D']['VITESSE_VEHICULE_ROUES'] * CV.KPH_TO_MS # HS2
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.yawRate = cp.vl['HS2_DYN_UCF_MDD_32D']['VITESSE_LACET_BRUTE'] * CV.DEG_TO_RAD # HS2
    ret.standstill = False  # TODO

    # gas
    ret.gas = cp.vl['Dyn_CMM']['P002_Com_rAPP'] / 99.5 # HS1
    ret.gasPressed = ret.gas > 0  # TODO

    # brake
    #ret.brake = cp.vl['HS2_DYN_UCF_2CD']['AUTO_BRAKING_PRESSURE'] / 50.6 # HS2 alternative
    ret.brake = cp.vl['Dyn2_FRE']['P515_Com_pStSpMstCyl'] / 1500.  # HS1
    ret.brakePressed = bool(cp.vl['Dat_BSI']['P013_MainBrake']) # HS1
    ret.parkingBrake = False # TODO bool(cp.vl['Dat_BSI']['PARKING_BRAKE']) is wrong signal

    # steering wheel
    ret.steeringAngleDeg = cp.vl['IS_DYN_VOL']['ANGLE_VOLANT'] # EPS
    ret.steeringRateDeg = cp.vl['IS_DYN_VOL']['VITESSE_ROT_VOL'] * cp.vl['IS_DYN_VOL']['SENS_ROT_VOL']  # EPS: Rotation speed * rotation sign/direction
    ret.steeringTorque = cp.vl['IS_DAT_DIRA']['CPLE_VOLANT']  # EPS: Driver torque
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)  # TODO: adjust threshold
    ret.steerFaultTemporary = False  # TODO
    ret.steerFaultPermanent = False  # TODO
    ret.espDisabled = False  # TODO

    # cruise
    # note: this is just for CC car not ACC right now
    ret.cruiseState.speed = cp.vl['HS2_DAT_MDD_CMD_452']['CONS_LIM_VITESSE_VEH'] * CV.KPH_TO_MS # HS2, set to 255 when ACC is off
    ret.cruiseState.enabled = cp.vl['HS2_DAT_MDD_CMD_452']['DDE_ACTIVATION_RVV_ACC'] == 1 # HS2
    ret.cruiseState.available = True  # TODO
    ret.cruiseState.nonAdaptive = cp.vl['HS2_DAT_MDD_CMD_452']['COCKPIT_GO_ACC_REQUEST'] == 0 # HS2, 0: ACC, 1: CC
    ret.cruiseState.standstill = False  # TODO
    ret.accFaulted = False

    # gear
    # reverse is same for automatic and manual
    if self.CP.transmissionType == TransmissionType.manual:
      ret.clutchPressed = bool(cp.vl['Dyn2_CMM']['P091_ConvCD_stDebVal']) # HS1
    if bool(cp.vl['Dat_BSI']['P103_Com_bRevGear']): # HS1
        ret.gearShifter = GearShifter.reverse
    else:
        ret.gearShifter = GearShifter.drive

    # TODO: safety
    ret.stockFcw = False
    ret.stockAeb = False

    # button presses
    blinker = cp.vl['HS2_DAT7_BSI_612']['CDE_CLG_ET_HDC'] # HS2
    ret.leftBlinker = blinker == 1
    ret.rightBlinker = blinker == 2

    # lock info
    ret.doorOpen = any([cp.vl['Dat_BSI']['DRIVER_DOOR'], cp.vl['Dat_BSI']['PASSENGER_DOOR']]) # HS1
    ret.seatbeltUnlatched = cp.vl['RESTRAINTS']['DRIVER_SEATBELT'] != 2

    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = [
      ('Dyn4_FRE', 50),
      ('Dat_BSI', 20),
      ('IS_DYN_VOL', 100),
      ('IS_DAT_DIRA', 10),
      ('Dyn2_FRE', 100),
      ('Dyn2_CMM', 50),
      ('Dyn_CMM', 100),
      ('RESTRAINTS', 10),
      ('HS2_DYN_ABR_38D', 25),
      ('HS2_DYN_UCF_MDD_32D', 50),
      # ('HS2_BGE_DYN5_CMM_228', 100), # TODO relevant? no signals in route
      ('HS2_DAT_MDD_CMD_452', 20),
      ('HS2_DAT7_BSI_612', 10),
    ]
    # adas_messages = [
    # ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0)
      # Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.adas], adas_messages, 1) #TODO: check CAN number
    }
