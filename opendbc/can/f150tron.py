#!/usr/bin/env python3
from cereal import car
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker

F150TRON_DBC = {
  'f150tron': CANDefine("opendbc/car/f150tron/f150tron.dbc"),
}

def get_can_parser(CP):
  signals = [
    # Lateral control signals
    ("SteeringAngle", "STEERING", 0),
    ("SteeringRate", "STEERING", 1),
    ("SteeringTorque", "STEERING", 2),

    # Longitudinal control signals
    ("AcceleratorPedal", "GAS", 0),
    ("BrakePedal", "BRAKE", 0),
    ("CruiseState", "CRUISE", 0),

    # Vehicle state signals
    ("VehicleSpeed", "VEHICLE_STATE", 0),
    ("YawRate", "VEHICLE_STATE", 1),
  ]

  checks = [
    ("STEERING", 50),
    ("GAS", 50),
    ("BRAKE", 50),
    ("CRUISE", 50),
    ("VEHICLE_STATE", 50),
  ]

  return CANParser("f150tron", signals, checks, 0)