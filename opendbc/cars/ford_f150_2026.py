import numpy as np

class CAR:
  NAME = "ford_f150_2026"
  DB = "ford_f150_2026"

  def __init__(self, CP, VM):
    self.CP = CP
    self.VM = VM

  def get_cam_can(self):
    return ["CAM_CAN"]

  def get_cam_bus(self):
    return 0

  def define(self):
    self.message_age = {}
    self.scc = None

    # Define signals
    self.add_message(0x100, 8, "steering_angle")
    self.add_message(0x101, 8, "gas_pedal")
    self.add_message(0x102, 8, "brake_pedal")
    self.add_message(0x103, 8, "speed")

    # Define bus
    self.add_bus(0, "CAM_CAN")

  def get_send_can(self, sendcan):
    # Define sendcan messages
    sendcan.add(0x200, 8, "steering_angle")
    sendcan.add(0x201, 8, "gas_pedal")
    sendcan.add(0x202, 8, "brake_pedal")

  def update(self, msg):
    # Update message age
    self.message_age[msg.address] = msg.timestamp

    # Update signals
    if msg.address == 0x100:
      self.VM["steering_angle"] = msg.signals["steering_angle"]
    elif msg.address == 0x101:
      self.VM["gas_pedal"] = msg.signals["gas_pedal"]
    elif msg.address == 0x102:
      self.VM["brake_pedal"] = msg.signals["brake_pedal"]
    elif msg.address == 0x103:
      self.VM["speed"] = msg.signals["speed"]

  def process_hud(self, hud):
    # Process HUD signals
    hud["steering_angle"] = self.VM["steering_angle"]
    hud["gas_pedal"] = self.VM["gas_pedal"]
    hud["brake_pedal"] = self.VM["brake_pedal"]
    hud["speed"] = self.VM["speed"]

  def process_control(self, CP):
    # Process control signals
    CP["steering_angle"] = self.VM["steering_angle"]
    CP["gas_pedal"] = self.VM["gas_pedal"]
    CP["brake_pedal"] = self.VM["brake_pedal"]

### FILE: opendbc/cars/tron.py