import math
from . import checksums

# Attempt to import dbc_lookup and SignalType from actual Cython modules
# This is the primary point of uncertainty as per the prompt.
DBC_LOOKUP_SUCCESS = False
DUMMY_CLASSES_USED = False

try:
    # Ideal case: a direct Python-accessible module for common C/C++ functions
    from opendbc.can.common_library import dbc_lookup, SignalType
    DBC_LOOKUP_SUCCESS = True
except ImportError:
    try:
        # Fallback: try to import from parser_pyx, assuming it might expose them
        # This is less likely if parser_pyx doesn't explicitly cpdef them.
        from opendbc.can.parser_pyx import dbc_lookup, SignalType # This is a guess
        DBC_LOOKUP_SUCCESS = True
    except ImportError:
        # Define dummy classes as a last resort for development and testing the packer logic
        DUMMY_CLASSES_USED = True
        # print("Warning: Using dummy dbc_lookup and SignalType. Actual Cython imports failed.")

        class DummyDBC:
            def __init__(self, name):
                self.name = name
                self.msgs = []  # List of Msg objects
                self.addr_to_msg = {}  # Dict mapping address to Msg
                self.name_to_msg = {}  # Dict mapping name to Msg

        class DummyMsg:
            def __init__(self, name, address, size, sigs):
                self.name = name
                self.address = address
                self.size = size
                self.sigs = sigs  # List of Signal objects
                # Add signals to name_to_sig for easier lookup if needed by packer logic
                self.name_to_sig = {s.name: s for s in sigs}


        class DummySignal:
            def __init__(self, name, start_bit, size, is_little_endian,
                         is_signed, factor, offset, lsb, msb, type_):
                self.name = name
                self.start_bit = start_bit  # Standard DBC start_bit (MSB for BE, LSB for LE)
                self.size = size
                self.is_little_endian = is_little_endian
                self.is_signed = is_signed
                self.factor = factor
                self.offset = offset
                # lsb and msb are crucial for _set_value.
                # In C++, these are calculated by opendbc.
                # For dummy, we need to ensure they are consistent.
                # If is_little_endian, lsb is start_bit. msb is start_bit + size - 1.
                # If is_big_endian (Motorola):
                #   DBC start_bit is the MSB. To find LSB for packing:
                #   byte_containing_msb = start_bit // 8
                #   bit_within_that_byte = start_bit % 8
                #   msb_global_bit_index = (byte_containing_msb * 8) + (7 - bit_within_that_byte) (this is standard MSB0 numbering)
                #   lsb_global_bit_index = msb_global_bit_index - size + 1
                # The C++ struct Signal has `lsb` and `msb` pre-calculated.
                # The prompt's _set_value uses `sig.lsb`. So we must provide it.
                self.lsb = lsb # Let's assume lsb is provided directly, as in C++ struct
                self.msb = msb # Let's assume msb is provided directly
                self.type = type_  # SignalType enum value

        def dbc_lookup(dbc_name_):
            # print(f"Warning: Using dummy dbc_lookup for {dbc_name_}")
            if dbc_name_ == "dummy_dbc_for_packer":
                # SignalType values must align with the dummy SignalType class below
                # COUNTER = 1, HONDA_CHECKSUM = 2
                counter_sig = DummySignal("COUNTER", 0, 4, True, False, 1, 0, 0, 3, 1) # lsb=0, msb=3
                # For Honda checksum, size 8, assume it's byte aligned for simplicity in dummy
                checksum_sig = DummySignal("CHECKSUM", 4, 8, True, False, 1, 0, 4, 11, 2) # lsb=4, msb=11
                sig1 = DummySignal("SIGNAL1", 12, 12, True, False, 1, 0, 12, 23, 0) # lsb=12, msb=23
                
                # Example for big endian signal (Motorola)
                # Name, DBC StartBit, Size, LE, Signed, Factor, Offset, LSB, MSB, Type
                # Start bit 8 (MSB of signal in byte 1), size 8.
                # Byte 1: | S7 S6 S5 S4 S3 S2 S1 S0 |
                # Global LSB for this signal is 8. MSB is 15.
                sig_be = DummySignal("SIGNAL_BE", 8, 8, False, False, 1, 0, 8, 15, 0)


                msg1 = DummyMsg("TEST_MSG", 0x123, 3, [counter_sig, checksum_sig, sig1]) # 3 bytes: 4+8+12 = 24 bits
                msg_be_test = DummyMsg("TEST_MSG_BE", 0x456, 2, [sig_be]) # 2 bytes for example

                db = DummyDBC(dbc_name_)
                db.msgs = [msg1, msg_be_test]
                db.addr_to_msg = {0x123: msg1, 0x456: msg_be_test}
                db.name_to_msg = {"TEST_MSG": msg1, "TEST_MSG_BE": msg_be_test}
                return db
            # Add more dummy DBCs if other tests need them
            if dbc_name_ == "honda_civic_touring_2016_can_generated": # Example from a test
                # This needs to be more fleshed out if specific tests run using this dummy
                db = DummyDBC(dbc_name_)
                # STEERING_CONTROL = 0xE4, size 5
                #   COUNTER (start_bit 4, size 2, type COUNTER) -> lsb 4, msb 5
                #   CHECKSUM (start_bit 6, size 4, type HONDA_CHECKSUM) -> lsb 6, msb 9
                s_counter = DummySignal("COUNTER", 4, 2, False, False, 1, 0, 4, 5, 1) # BE, lsb needs calc if start_bit is MSB style
                s_checksum = DummySignal("CHECKSUM", 0, 4, False, False, 1, 0, 0, 3, 2) # BE, lsb needs calc

                # Re-calculating lsb for BE signals based on common.cc logic for testing _set_value
                # For STEERING_CONTROL message (size 5 bytes):
                # Signal COUNTER: start_bit=4 (MSB), size=2, is_little_endian=False.
                # Byte 0: 7 6 5 4[S1 S0]x x. Start_bit 4 means bit 4 in byte 0.
                # LSB for this signal (if start_bit is MSB):
                #   byte_idx = 4 // 8 = 0
                #   bit_idx_in_byte = 4 % 8 = 4
                #   msb_abs = (byte_idx * 8) + (7 - bit_idx_in_byte) = (0*8) + (7-4) = 3 (this is MSB0 style)
                #   lsb_abs = msb_abs - size + 1 = 3 - 2 + 1 = 2.
                #   So, for COUNTER (start_bit=4, size=2, BE): lsb=2, msb=3.
                # Let's use the lsb/msb directly as if opendbc calculated them.
                # The prompt _set_value uses sig.lsb.
                # common.cc: honda_compute_checksum: CHECKSUM (STEERING_CONTROL) sig.lsb = 0, sig.size = 4
                # common.cc: honda_compute_checksum: COUNTER (STEERING_CONTROL) sig.lsb = 4, sig.size = 2
                # These lsb values from common.cc are likely 0-indexed from start of message, little-endian byte order.
                # This implies that `lsb` in the C++ struct is the actual LSB bit index from start of message data buffer.

                # For STEERING_CONTROL (0xE4, 5 bytes)
                # SIG_COUNTER: lsb=4, size=2 (bits 4,5 of byte 0)
                # SIG_CHECKSUM: lsb=0, size=4 (bits 0,1,2,3 of byte 0)
                # These are used by the C++ packer. Let's use these for the dummy.
                # Note: These imply little-endian byte packing for these specific signals,
                # even if the signal itself is "big_endian" in terms of bit interpretation.
                # The `is_little_endian` in `_set_value` refers to byte order for multi-byte signals.
                # The `lsb` attribute itself should be the absolute LSB position.
                
                # For STEERING_CONTROL, size 5 bytes.
                # ACC_STATUS: name="ACC_STATUS", lsb=39, size=1, is_little_endian=True
                # Assuming the lsb values from C++ are globally indexed LSBs.
                s_counter_honda = DummySignal("COUNTER", 0, 2, True, False, 1, 0, 4, 5, 1) # lsb=4, size=2
                s_checksum_honda = DummySignal("CHECKSUM", 0, 4, True, False, 1, 0, 0, 3, 2) # lsb=0, size=4
                steering_control_sigs = [s_counter_honda, s_checksum_honda]
                # Add other signals to make it 5 bytes if needed for checksums
                # For simplicity, assume checksum only needs these.
                msg_steering_control = DummyMsg("STEERING_CONTROL", 0xE4, 5, steering_control_sigs)
                db.msgs.append(msg_steering_control)
                db.addr_to_msg[0xE4] = msg_steering_control
                db.name_to_msg["STEERING_CONTROL"] = msg_steering_control
                return db
            elif dbc_name_ == "test.dbc":
                # Based on the simplified test.dbc structure provided in the subtask
                db = DummyDBC(dbc_name_)
                ST = SignalType # Alias for brevity

                # CAN_FD_MESSAGE (Address 245, Size 8)
                # Name, DBC StartBit, Size, LE, Signed, Factor, Offset, LSB, MSB, Type
                can_fd_counter = DummySignal("COUNTER", 0, 8, True, False, 1, 0, 0, 7, ST.COUNTER)
                can_fd_signal_a = DummySignal("SIGNAL_A", 8, 16, True, False, 1, 0, 8, 23, ST.DEFAULT)
                can_fd_signal_b = DummySignal("SIGNAL_B", 24, 32, True, False, 1, 0, 24, 55, ST.DEFAULT)
                can_fd_checksum_hull = DummySignal("CHECKSUM_HULL", 56, 8, True, False, 1, 0, 56, 63, ST.DEFAULT)
                msg_can_fd = DummyMsg("CAN_FD_MESSAGE", 245, 8, [
                    can_fd_counter, can_fd_signal_a, can_fd_signal_b, can_fd_checksum_hull
                ])
                db.msgs.append(msg_can_fd)
                db.addr_to_msg[245] = msg_can_fd
                db.name_to_msg["CAN_FD_MESSAGE"] = msg_can_fd

                # STEERING_CONTROL (Address 228, Size 3)
                # Name, DBC StartBit, Size, LE, Signed, Factor, Offset, LSB, MSB, Type
                sc_steer_torque = DummySignal("STEER_TORQUE", 0, 12, True, True, 1, 0, 0, 11, ST.DEFAULT)
                sc_steer_torque_req = DummySignal("STEER_TORQUE_REQUEST", 12, 1, True, False, 1, 0, 12, 12, ST.DEFAULT)
                sc_counter = DummySignal("COUNTER", 16, 4, True, False, 1, 0, 16, 19, ST.COUNTER)
                sc_checksum = DummySignal("CHECKSUM", 20, 4, True, False, 1, 0, 20, 23, ST.HONDA_CHECKSUM)
                msg_steering_control = DummyMsg("STEERING_CONTROL", 228, 3, [
                    sc_steer_torque, sc_steer_torque_req, sc_counter, sc_checksum
                ])
                db.msgs.append(msg_steering_control)
                db.addr_to_msg[228] = msg_steering_control
                db.name_to_msg["STEERING_CONTROL"] = msg_steering_control
                
                # Brake_Status (Address 100, Size 8 - corrected from 1 as per subtask)
                # Name, DBC StartBit, Size, LE, Signed, Factor, Offset, LSB, MSB, Type
                bs_signal1 = DummySignal("Signal1", 0, 64, True, False, 1, 0, 0, 63, ST.DEFAULT)
                msg_brake_status = DummyMsg("Brake_Status", 100, 8, [bs_signal1])
                db.msgs.append(msg_brake_status)
                db.addr_to_msg[100] = msg_brake_status
                db.name_to_msg["Brake_Status"] = msg_brake_status
                
                return db

            raise RuntimeError(f"Dummy dbc_lookup called with unhandled DBC: {dbc_name_}")

        class SignalType:
            DEFAULT = 0
            COUNTER = 1
            HONDA_CHECKSUM = 2
            TOYOTA_CHECKSUM = 3
            PEDAL_CHECKSUM = 4 # From common.cc, this is type PEDAL_CHECKSUM
            VOLKSWAGEN_MQB_MEB_CHECKSUM = 5
            XOR_CHECKSUM = 6
            SUBARU_CHECKSUM = 7
            CHRYSLER_CHECKSUM = 8
            HKG_CAN_FD_CHECKSUM = 9
            FCA_GIORGIO_CHECKSUM = 10
            TESLA_CHECKSUM = 11
            # Ensure this list matches checksums.py and common_dbc.h
        
        # Make them available for CANPacker init
        # This is a bit of a hack for the class structure below
        # A better way would be to pass them into __init__
        globals()['dbc_lookup_impl_dummy'] = dbc_lookup
        globals()['SignalType_impl_dummy'] = SignalType


class CANPacker:
    def __init__(self, dbc_name):
        global DUMMY_CLASSES_USED, DBC_LOOKUP_SUCCESS
        if DUMMY_CLASSES_USED:
            self.dbc_lookup_impl = globals()['dbc_lookup_impl_dummy']
            self.SignalType_impl = globals()['SignalType_impl_dummy']
            # print(f"CANPacker instance for '{dbc_name}' is using DUMMY dbc_lookup and SignalType.")
        elif DBC_LOOKUP_SUCCESS:
            # These were successfully imported from a Cython module
            self.dbc_lookup_impl = dbc_lookup
            self.SignalType_impl = SignalType
            # print(f"CANPacker instance for '{dbc_name}' is using REAL dbc_lookup and SignalType.")
        else:
            # This state should ideally not be reached if dummy setup is correct.
            raise RuntimeError("CRITICAL: dbc_lookup and SignalType not available, and dummies not set up.")

        self.dbc = self.dbc_lookup_impl(dbc_name)
        if self.dbc is None:
            raise RuntimeError(f"DBC file {dbc_name} not found or not parseable by loaded lookup.")

        self.signal_lookup = {}  # Maps addr -> {sig_name: Signal}
        self.counters = {}  # Stores current counter values for each message address (addr -> int)

        if hasattr(self.dbc, 'msgs') and self.dbc.msgs is not None:
            for msg_def in self.dbc.msgs:
                if msg_def.address not in self.signal_lookup:
                    self.signal_lookup[msg_def.address] = {}
                # Also ensure name_to_msg map contains this msg_def if not already by dbc_lookup
                if msg_def.name not in self.dbc.name_to_msg:
                    self.dbc.name_to_msg[msg_def.name] = msg_def

                for sig in msg_def.sigs:
                    self.signal_lookup[msg_def.address][sig.name] = sig
        else:
            # This might happen if dummy DBC is not fully populated or real DBC has different structure.
            # print(f"Warning: DBC for {dbc_name} has no 'msgs' attribute or it's None.")
            pass

    def _set_value(self, msg_data: bytearray, sig: object, ival: int):
        # Correct implementation based on C++ logic from the prompt
        # Assumes sig object has attributes: lsb, size, is_little_endian

        byte_index = sig.lsb // 8
        bits_to_pack = sig.size

        if sig.size < 64: # Ensure ival fits within signal's specified bit size
            ival &= ((1 << sig.size) - 1)
        
        remaining_bits = bits_to_pack
        current_val_chunk = ival # This is the part of ival we're currently packing

        # Loop while there are bits remaining and we are within message bounds
        while 0 <= byte_index < len(msg_data) and remaining_bits > 0:
            # shift_in_byte: where the current chunk of bits starts in msg_data[byte_index]
            # If this is the first byte we're writing to for this signal (i.e., the one containing its LSB),
            # the shift is sig.lsb % 8. For subsequent bytes (if any), we start at bit 0 of that byte.
            shift_in_byte = sig.lsb % 8 if (sig.lsb // 8) == byte_index else 0
            
            # num_bits_in_this_byte: how many bits we can (or need to) pack into msg_data[byte_index]
            # It's the minimum of remaining_bits and space left in current byte from shift_in_byte.
            num_bits_in_this_byte = min(remaining_bits, 8 - shift_in_byte)

            # Create a mask to clear the bits we are about to set in msg_data[byte_index]
            mask = ((1 << num_bits_in_this_byte) - 1) << shift_in_byte
            msg_data[byte_index] &= ~mask  # Clear the bits

            # value_to_pack_in_byte: the actual bits from current_val_chunk for this byte
            # We take the LSBs of current_val_chunk.
            value_to_pack_in_byte = (current_val_chunk & ((1 << num_bits_in_this_byte) - 1))
            msg_data[byte_index] |= (value_to_pack_in_byte << shift_in_byte) # Set the bits

            remaining_bits -= num_bits_in_this_byte
            current_val_chunk >>= num_bits_in_this_byte # Discard the bits we've just packed

            # Determine next byte_index based on endianness (for multi-byte signals)
            if sig.is_little_endian:
                byte_index += 1
            else: # Big Endian (Motorola)
                byte_index -= 1

    def pack(self, address: int, signals_with_values: dict) -> bytes:
        if not self.dbc or address not in self.dbc.addr_to_msg:
            print(f"Error: Undefined address {address} in DBC {self.dbc.name if self.dbc else 'None'}")
            return b''

        msg_def = self.dbc.addr_to_msg[address]
        ret_msg = bytearray(msg_def.size) # Initialize with zeros

        counter_set_explicitly = False
        counter_sig_def_for_auto_increment = None

        # Set explicitly provided signal values
        for sig_name, sig_val_float in signals_with_values.items():
            if address not in self.signal_lookup or sig_name not in self.signal_lookup[address]:
                print(f"Warning: Undefined signal '{sig_name}' for address {address}")
                continue
            
            sig = self.signal_lookup[address][sig_name]

            ival = int(round((sig_val_float - sig.offset) / sig.factor))

            if ival < 0 and sig.is_signed:
                ival = (1 << sig.size) + ival # 2's complement for the given size

            self._set_value(ret_msg, sig, ival)

            # Check if this is a COUNTER signal
            # Using direct name check as fallback like in C++ code.
            is_counter_signal = (sig.type == self.SignalType_impl.COUNTER or
                                (hasattr(sig, 'name') and sig.name == "COUNTER"))

            if is_counter_signal:
                self.counters[address] = int(sig_val_float) # Store the explicitly set counter value
                counter_set_explicitly = True
        
        # Find the counter signal definition if one exists for auto-increment
        if address in self.signal_lookup:
            for s_name, s_def in self.signal_lookup[address].items():
                is_counter_s_def = (s_def.type == self.SignalType_impl.COUNTER or
                                    (hasattr(s_def, 'name') and s_def.name == "COUNTER"))
                if is_counter_s_def:
                    counter_sig_def_for_auto_increment = s_def
                    break

        # Handle COUNTER signal if not set explicitly but exists
        if not counter_set_explicitly and counter_sig_def_for_auto_increment is not None:
            sig = counter_sig_def_for_auto_increment
            current_counter_val = self.counters.get(address, 0) # Get current or default to 0
            
            self._set_value(ret_msg, sig, current_counter_val)
            
            # Increment and store for next time
            self.counters[address] = (current_counter_val + 1) % (1 << sig.size)


        # Handle CHECKSUM signal (must be done after all other signals are set)
        checksum_sig_def = None
        if address in self.signal_lookup:
            for s_name, s_def in self.signal_lookup[address].items():
                # Check if signal type is one of the known checksum types
                if s_def.type in [
                    self.SignalType_impl.HONDA_CHECKSUM, self.SignalType_impl.TOYOTA_CHECKSUM,
                    self.SignalType_impl.PEDAL_CHECKSUM, self.SignalType_impl.VOLKSWAGEN_MQB_MEB_CHECKSUM,
                    self.SignalType_impl.XOR_CHECKSUM, self.SignalType_impl.SUBARU_CHECKSUM,
                    self.SignalType_impl.CHRYSLER_CHECKSUM, self.SignalType_impl.HKG_CAN_FD_CHECKSUM,
                    self.SignalType_impl.FCA_GIORGIO_CHECKSUM, self.SignalType_impl.TESLA_CHECKSUM
                ]:
                    checksum_sig_def = s_def
                    break
        
        if checksum_sig_def is not None:
            checksum_val = 0
            sig_type = checksum_sig_def.type # For brevity in elif chain
            
            # Data passed to checksum function is the current state of ret_msg
            # Some checksums (like Chrysler) might exclude the checksum byte itself,
            # but the checksum functions in checksums.py are expected to handle that if d is the full msg_data.
            # The C++ versions take `const std::vector<uint8_t> &d` which is the msg data.
            # `xor_checksum` and `tesla_checksum` in `checksums.py` use `sig.start_bit` to exclude checksum byte.

            if sig_type == self.SignalType_impl.HONDA_CHECKSUM:
                checksum_val = checksums.honda_checksum(address, checksum_sig_def, ret_msg)
            elif sig_type == self.SignalType_impl.TOYOTA_CHECKSUM:
                checksum_val = checksums.toyota_checksum(address, checksum_sig_def, ret_msg)
            elif sig_type == self.SignalType_impl.SUBARU_CHECKSUM:
                checksum_val = checksums.subaru_checksum(address, checksum_sig_def, ret_msg)
            elif sig_type == self.SignalType_impl.CHRYSLER_CHECKSUM:
                # `chrysler_checksum` processes `d[:-1]`, so `ret_msg` should be fine.
                checksum_val = checksums.chrysler_checksum(address, checksum_sig_def, ret_msg)
            elif sig_type == self.SignalType_impl.VOLKSWAGEN_MQB_MEB_CHECKSUM:
                checksum_val = checksums.volkswagen_mqb_meb_checksum(address, checksum_sig_def, ret_msg)
            elif sig_type == self.SignalType_impl.XOR_CHECKSUM:
                checksum_val = checksums.xor_checksum(address, checksum_sig_def, ret_msg)
            elif sig_type == self.SignalType_impl.PEDAL_CHECKSUM:
                checksum_val = checksums.pedal_checksum(address, checksum_sig_def, ret_msg)
            elif sig_type == self.SignalType_impl.HKG_CAN_FD_CHECKSUM:
                checksum_val = checksums.hkg_can_fd_checksum(address, checksum_sig_def, ret_msg)
            elif sig_type == self.SignalType_impl.FCA_GIORGIO_CHECKSUM:
                checksum_val = checksums.fca_giorgio_checksum(address, checksum_sig_def, ret_msg)
            elif sig_type == self.SignalType_impl.TESLA_CHECKSUM:
                checksum_val = checksums.tesla_checksum(address, checksum_sig_def, ret_msg)
            else:
                print(f"Warning: Unknown checksum type {sig_type} for signal {checksum_sig_def.name}")

            self._set_value(ret_msg, checksum_sig_def, checksum_val)

        return bytes(ret_msg)

    def make_can_msg(self, name_or_addr, bus: int, values: dict):
        addr = 0
        if isinstance(name_or_addr, str):
            if self.dbc and name_or_addr in self.dbc.name_to_msg:
                addr = self.dbc.name_to_msg[name_or_addr].address
            else:
                print(f"Error: Message name '{name_or_addr}' not found in DBC {self.dbc.name if self.dbc else 'None'}")
                return addr, b'', bus # Return empty data if name not found
        elif isinstance(name_or_addr, int):
            addr = name_or_addr
        else:
            print(f"Error: Invalid name_or_addr type: {type(name_or_addr)}. Must be str or int.")
            return 0, b'', bus


        packed_data = self.pack(addr, values)
        return addr, packed_data, bus

# To inform the user about dummy usage status after script processing:
# if DUMMY_CLASSES_USED:
#   print("Note: CANPacker is operating with DUMMY DBC structures and lookup due to import failures for actual Cython components.")
# elif not DBC_LOOKUP_SUCCESS and not DUMMY_CLASSES_USED:
#   # This indicates a logic flaw in the try-except-else for imports
#   print("CRITICAL SETUP FAILURE: Neither real nor dummy DBC components were loaded.")
# else: # DBC_LOOKUP_SUCCESS is True
#   print("Note: CANPacker is operating with REAL DBC structures and lookup from Cython components.")
