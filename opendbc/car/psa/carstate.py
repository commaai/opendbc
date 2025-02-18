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
    ret.yawRate = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VITESSE_LACET_BRUTE'] * CV.DEG_TO_RAD # HS2
    ret.standstill = bool(cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VEHICLE_STANDSTILL']) # steering possible down to standstill

    # gas
    ret.gas = cp_main.vl['DRIVER']['GAS_PEDAL'] / 99.5 # HS1
    ret.gasPressed = ret.gas > 0

    # brake
    ret.brake = cp.vl['Dyn2_FRE']['BRAKE_PRESSURE'] / 1500.  # HS1
    ret.brakePressed = bool(cp_main.vl['Dat_BSI']['P013_MainBrake']) # HS1
    ret.parkingBrake = cp.vl['Dyn_EasyMove']['P337_Com_stPrkBrk'] == 1 # 0: disengaged, 1: engaged, 3: brake actuator moving
    # TODO (corsa only?) bool(cp_main.vl['Dat_BSI']['PARKING_BRAKE'])

    # steering wheel
    ret.steeringAngleDeg = cp.vl['STEERING_ALT']['ANGLE'] # EPS
    ret.steeringRateDeg = cp.vl['STEERING_ALT']['RATE'] * cp.vl['STEERING_ALT']['RATE_SIGN']  # EPS: Rotation speed * rotation sign/direction
    ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)  # TODO: adjust threshold
    ret.steerFaultTemporary = False  # TODO
    ret.steerFaultPermanent = False  # TODO
    ret.espDisabled = bool(cp_adas.vl['ESP']['ESP_STATUS_INV'])

    # cruise
    # note: this is just for ACC car not CC right now
    ret.cruiseState.speed = cp_adas.vl['HS2_DAT_MDD_CMD_452']['CONS_LIM_VITESSE_VEH'] * CV.KPH_TO_MS # HS2, set to 255 when ACC is off, -2 kph offset from dashboard speed
    ret.cruiseState.enabled = cp_adas.vl['HS2_DAT_MDD_CMD_452']['DDE_ACTIVATION_RVV_ACC'] == 1 # HS2
    ret.cruiseState.available = cp_adas.vl['HS2_DYN1_MDD_ETAT_2B6']['ACC_STATUS'] > 2 # HS2
    ret.cruiseState.nonAdaptive = cp_adas.vl['HS2_DAT_MDD_CMD_452']['TYPE_REGUL_LONGI'] != 3 # HS2, 0: None, 1: CC, 2: Limiter, 3: ACC
    ret.cruiseState.standstill = bool(cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VEHICLE_STANDSTILL'])
    ret.accFaulted = False

    # gear
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
    blinker = cp_main.vl['HS2_DAT7_BSI_612']['CDE_CLG_ET_HDC'] # HS1
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
      ('Dyn_EasyMove', 50),
    ]
    adas_messages = [
      ('ESP', 50),
      ('HS2_DYN_ABR_38D', 25),
      ('HS2_DYN_UCF_MDD_32D', 50),
      ('HS2_DAT_MDD_CMD_452', 20),
      ('HS2_DYN1_MDD_ETAT_2B6', 50),
    ]
    main_messages = [
      ('Dat_BSI', 20),
      ('RESTRAINTS', 10),
      ('DRIVER', 10),
      ('HS2_DAT7_BSI_612', 10),
    ]
    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.main], main_messages, 2),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.adas], adas_messages, 1),
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
    }
