#!/usr/bin/env python3
"""
Simple Signal Extractor for CAN messages.

Extracts:
- Vehspeed (Vehicle Speed)
- ReverseGear (Reverse Gear Indicator)
- Illumination (Lighting Status)
- SWC_Buttons (Steering Wheel Control Buttons)
"""

import time
import sys
from opendbc.can import CANParser
from opendbc.can.dbc import DBC
from opendbc import DBC_PATH
import os


class SignalExtractor:
    """Simple extractor for common vehicle signals."""
    
    def __init__(self, dbc_name: str, bus: int = 0):
        self.dbc_name = dbc_name
        self.bus = bus
        self.dbc = DBC(dbc_name)
        
        # Find which messages have our target signals
        self.signal_mapping = self._find_signals()
        
        # Build message list for parser
        messages = [(msg_name, 10) for msg_name in self.signal_mapping.keys()]
        
        if not messages:
            print(f"[WARNING] No target signals found in {dbc_name}")
            print("   The extractor will still work but may return None for all signals.")
            messages = []
        
        # Create parser - None if no messages found
        self.parser = CANParser(dbc_name, messages, bus) if messages else None
        
        print(f"[OK] Extractor initialized for {dbc_name}")
        print(f"[INFO] Found {len(messages)} messages with target signals")
    
    def _find_signals(self):
        """Search for target signals in the DBC file."""
        # These are the signal name patterns we're looking for
        target_signals = {
            'vehspeed': ['vehspeed', 'vehiclespeed', 'vehicle_speed', 'speed', 'car_speed', 'hudspeedset', 'speed_unit'],
            'reversegear': ['reversegear', 'reverse_gear', 'gear', 'gearshifter', 'prndl', 'gearsel', 'swigear'],
            'illumination': ['illumination', 'lights', 'lighting', 'lights_on', 'alightstat', 'alight', 'lightstat'],
            'swc_buttons': ['swc_buttons', 'swc', 'steering_wheel', 'cruise_buttons', 'buttons', 'btn', 'switch', 'cruise', 'steering']
        }
        
        signal_mapping = {}
        
        # Go through all messages in the DBC
        for msg_addr, msg in self.dbc.msgs.items():
            found_signals = {}
            
            # Check each signal in this message
            for sig_name, sig in msg.sigs.items():
                sig_lower = sig_name.lower()
                
                # See if this signal matches any of our target categories
                for target_key, variations in target_signals.items():
                    if any(var in sig_lower for var in variations):
                        if target_key not in found_signals:
                            found_signals[target_key] = []
                        found_signals[target_key].append(sig_name)
            
            # If we found any signals, add this message to our mapping
            if found_signals:
                signal_mapping[msg.name] = found_signals
                print(f"  [MSG] Message: {msg.name} (0x{msg_addr:X})")
                for key, signals in found_signals.items():
                    print(f"     - {key}: {signals}")
        
        return signal_mapping
    
    def update(self, can_data):
        """Update parser with new CAN data."""
        if self.parser is None:
            return self.extract()
        
        self.parser.update(can_data)
        return self.extract()
    
    def extract(self):
        """Extract all target signals from the parser."""
        extracted = {
            'vehspeed': None,
            'reversegear': None,
            'illumination': None,
            'swc_buttons': None
        }
        
        if self.parser is None:
            return extracted
        
        # Go through each message we're monitoring
        for msg_name, signals in self.signal_mapping.items():
            try:
                msg_data = self.parser.vl[msg_name]
                
                # Extract each signal type from this message
                for signal_type, signal_names in signals.items():
                    # Only take the first one we find (usually the primary signal)
                    if extracted[signal_type] is None:
                        for sig_name in signal_names:
                            if sig_name in msg_data:
                                extracted[signal_type] = msg_data[sig_name]
                                break
            except KeyError:
                # Message not received yet, skip it
                continue
        
        return extracted
    
    def print_extracted(self, extracted=None, clear_previous=False):
        """Print extracted signals."""
        if extracted is None:
            extracted = self.extract()
        
        if clear_previous:
            # Clear the previous 9 lines
            for _ in range(9):
                sys.stdout.write('\033[2K\033[1A')
            sys.stdout.flush()
        
        # Color codes
        RESET = '\033[0m'
        BOLD = '\033[1m'
        CYAN = '\033[96m'
        GREEN = '\033[92m'
        YELLOW = '\033[93m'
        MAGENTA = '\033[95m'
        BLUE = '\033[94m'
        DIM = '\033[2m'
        
        # Format the values
        if extracted['vehspeed'] is not None:
            speed_str = f"{GREEN}{BOLD}{extracted['vehspeed']:.2f} km/h{RESET}"
        else:
            speed_str = f"{DIM}N/A{RESET}"
        
        if extracted['reversegear'] is not None:
            gear_val = extracted['reversegear']
            gear_str = f"{MAGENTA}{BOLD}{'ON' if gear_val else 'OFF'}{RESET} ({gear_val})"
        else:
            gear_str = f"{DIM}N/A{RESET}"
        
        if extracted['illumination'] is not None:
            illum_val = extracted['illumination']
            light_str = f"{YELLOW}{BOLD}{'ON' if illum_val else 'OFF'}{RESET} ({illum_val})"
        else:
            light_str = f"{DIM}N/A{RESET}"
        
        if extracted['swc_buttons'] is not None:
            btn_str = f"{BLUE}{BOLD}{extracted['swc_buttons']}{RESET}"
        else:
            btn_str = f"{DIM}N/A{RESET}"
        
        # Print the box
        print(f"{CYAN}╔{'═'*66}╗{RESET}")
        print(f"{CYAN}║{RESET} {BOLD}EXTRACTED SIGNALS{RESET} " + " " * 48 + f"{CYAN}║{RESET}")
        print(f"{CYAN}╠{'═'*66}╣{RESET}")
        print(f"{CYAN}║{RESET} {GREEN}[SPEED]{RESET}     Vehspeed:      {speed_str:>25} {CYAN}║{RESET}")
        print(f"{CYAN}║{RESET} {MAGENTA}[GEAR]{RESET}      ReverseGear:   {gear_str:>25} {CYAN}║{RESET}")
        print(f"{CYAN}║{RESET} {YELLOW}[LIGHT]{RESET}     Illumination:  {light_str:>25} {CYAN}║{RESET}")
        print(f"{CYAN}║{RESET} {BLUE}[BTN]{RESET}       SWC_Buttons:   {btn_str:>25} {CYAN}║{RESET}")
        print(f"{CYAN}╚{'═'*66}╝{RESET}")
        sys.stdout.flush()


def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python signal_extractor.py <dbc_file_name>")
        print("\nAvailable DBC files:")
        dbc_files = [f.replace('.dbc', '') for f in os.listdir(DBC_PATH) if f.endswith('.dbc')]
        for f in sorted(dbc_files)[:10]:
            print(f"  - {f}")
        if len(dbc_files) > 10:
            print(f"  ... and {len(dbc_files) - 10} more")
        return
    
    dbc_name = sys.argv[1]
    # Remove .dbc if they included it
    if dbc_name.endswith('.dbc'):
        dbc_name = dbc_name[:-4]
    
    try:
        extractor = SignalExtractor(dbc_name, bus=0)
        print("\n[TIP] Use the simulator to test the extractor:")
        print("   python can_simulator.py")
    except Exception as e:
        print(f"[ERROR] Error: {e}")


if __name__ == '__main__':
    main()
