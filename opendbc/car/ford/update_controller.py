
path = "carcontroller.py"
content = open(path, encoding="utf-8").read()

if "add_mac" not in content:
    # 1. Add import
    content = "from opendbc.car.secoc import add_mac\n" + content
    
    # 2. Add SecOC counters in __init__
    init_target = "self.distance_bar_frame = 0"
    init_replacement = "self.distance_bar_frame = 0\n    self.secoc_lat_message_counter = 0\n    self.secoc_acc_message_counter = 0"
    content = content.replace(init_target, init_replacement)

    # 3. Wrap CAN messages with add_mac if SECOC is enabled
    lat_msg_target = "can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0., 0., -self.apply_curvature_last, 0., counter))"
    lat_msg_replacement = """can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0., 0., -self.apply_curvature_last, 0., counter))
      if self.CP.flags & FordFlags.SECOC:
        # TODO: wire up real sync values from CarState
        can_sends[-1] = add_mac(self.secoc_key, 0, 0, self.secoc_lat_message_counter, can_sends[-1])
        self.secoc_lat_message_counter += 1"""
    content = content.replace(lat_msg_target, lat_msg_replacement)

    open(path, "w", encoding="utf-8").write(content)
    print("Successfully updated carcontroller.py!")
else:
    print("Carcontroller already updated!")

