from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.psacan import CanBus
from opendbc.car.psa.values import DBC, CarControllerParams
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_adas = can_parsers[Bus.adas]
    cp_main = can_parsers[Bus.main]
    ret = structs.CarState()

    # car speed
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'], # HS1
      cp.vl['Dyn4_FRE']['P264_VehV_VPsvValWhlFrtR'],
      cp.vl['Dyn4_FRE']['P265_VehV_VPsvValWhlBckL'],
      cp.vl['Dyn4_FRE']['P266_VehV_VPsvValWhlBckR'],
    )
    ret.vEgoRaw = cp_adas.vl['HS2_DYN_ABR_38D']['VITESSE_VEHICULE_ROUES'] * CV.KPH_TO_MS # HS2
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.yawRate = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VITESSE_LACET_BRUTE'] * CV.DEG_TO_RAD # HS2 TODO: only on BUS1
    ret.standstill = False  # TODO

    # gas
    ret.gas = cp_main.vl['DRIVER']['GAS_PEDAL'] / 99.5 # HS1
    ret.gasPressed = ret.gas > 0  # TODO

    # brake
    #ret.brake = cp.vl['HS2_DYN_UCF_2CD']['AUTO_BRAKING_PRESSURE'] / 50.6 # HS2 alternative
    ret.brake = cp.vl['Dyn2_FRE']['BRAKE_PRESSURE'] / 1500.  # HS1
    ret.brakePressed = bool(cp_main.vl['Dat_BSI']['P013_MainBrake']) # HS1
    ret.parkingBrake = False # TODO bool(cp_main.vl['Dat_BSI']['PARKING_BRAKE']) is wrong signal

    # steering wheel
    ret.steeringAngleDeg = cp.vl['STEERING_ALT']['ANGLE'] # EPS
    ret.steeringRateDeg = cp.vl['STEERING_ALT']['RATE'] * cp.vl['STEERING_ALT']['RATE_SIGN']  # EPS: Rotation speed * rotation sign/direction
    ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']  # EPS: Driver torque
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)  # TODO: adjust threshold
    ret.steerFaultTemporary = False  # TODO
    ret.steerFaultPermanent = False  # TODO
    ret.espDisabled = False  # TODO

    # cruise
    # note: this is just for CC car not ACC right now
    ret.cruiseState.speed = cp_adas.vl['HS2_DAT_MDD_CMD_452']['CONS_LIM_VITESSE_VEH'] * CV.KPH_TO_MS # HS2, set to 255 when ACC is off
    ret.cruiseState.enabled = cp_adas.vl['HS2_DAT_MDD_CMD_452']['DDE_ACTIVATION_RVV_ACC'] == 1 # HS2
    ret.cruiseState.available = True  # TODO
    ret.cruiseState.nonAdaptive = False # cp_adas.vl['HS2_DAT_MDD_CMD_452']['COCKPIT_GO_ACC_REQUEST'] == 0 # HS2, 0: CC, 1: ACC, no signal in route
    ret.cruiseState.standstill = False  # TODO
    ret.accFaulted = False

    # gear
    # reverse is same for automatic and manual
    if self.CP.transmissionType == TransmissionType.manual:
      ret.clutchPressed = bool(cp.vl['Dyn2_CMM']['P091_ConvCD_stDebVal']) # HS1
    if bool(cp_main.vl['Dat_BSI']['P103_Com_bRevGear']): # HS1
        ret.gearShifter = GearShifter.reverse
    else:
        ret.gearShifter = GearShifter.drive

    # TODO: safety
    ret.stockFcw = False
    ret.stockAeb = False

    # button presses
    blinker = cp_adas.vl['HS2_DAT7_BSI_612']['CDE_CLG_ET_HDC'] # HS2
    ret.leftBlinker = blinker == 1
    ret.rightBlinker = blinker == 2

    # lock info
    ret.doorOpen = any([cp_main.vl['Dat_BSI']['DRIVER_DOOR'], cp_main.vl['Dat_BSI']['PASSENGER_DOOR']]) # HS1
    ret.seatbeltUnlatched = cp_main.vl['RESTRAINTS']['DRIVER_SEATBELT'] != 2

    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = [
      ('Dyn4_FRE', 50),
      ('STEERING_ALT', 100),
      ('STEERING', 100),
      ('Dyn2_FRE', 100),
      ('Dyn2_CMM', 50),
    ]
    adas_messages = [
      ('HS2_DYN_ABR_38D', 25),
      ('HS2_DYN_UCF_MDD_32D', 50),
      ('HS2_DAT_MDD_CMD_452', 20),
      ('HS2_DAT7_BSI_612', 10),
    ]
    main_messages = [
      ('Dat_BSI', 20),
      ('RESTRAINTS', 10),
      ('DRIVER', 10),
    ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 2),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.adas], adas_messages, 1),
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.main], main_messages, 0)
    }
