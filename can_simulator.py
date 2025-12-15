#!/usr/bin/env python3
"""
CAN Bus Simulator for testing signal extractor.

Generates simulated CAN messages with:
- Vehspeed (Vehicle Speed)
- ReverseGear (Reverse Gear Indicator)
- Illumination (Lighting Status)
- SWC_Buttons (Steering Wheel Control Buttons)
"""

import time
import sys
from signal_extractor import SignalExtractor
from opendbc.can import CANPacker


class CANSimulator:
    """Simple CAN bus simulator for testing."""
    
    def __init__(self, dbc_name: str, bus: int = 0):
        # ANSI codes for terminal stuff
        self.CLEAR_LINE = '\033[2K'
        self.CURSOR_UP = '\033[1A'
        self.CURSOR_HOME = '\033[H'
        self.CLEAR_SCREEN = '\033[2J'
        self.RESET_CURSOR = '\033[0;0H'
        self.SAVE_CURSOR = '\033[s'
        self.RESTORE_CURSOR = '\033[u'
        
        # Colors
        self.RESET = '\033[0m'
        self.BOLD = '\033[1m'
        self.DIM = '\033[2m'
        self.BLUE = '\033[94m'
        self.GREEN = '\033[92m'
        self.YELLOW = '\033[93m'
        self.RED = '\033[91m'
        self.CYAN = '\033[96m'
        self.MAGENTA = '\033[95m'
        self.WHITE = '\033[97m'
        # Background colors (not really used but here anyway)
        self.BG_BLUE = '\033[44m'
        self.BG_GREEN = '\033[42m'
        self.BG_DARK = '\033[100m'
        
        self.lines_printed = 0  # Keep track for clearing
        
        self.dbc_name = dbc_name
        self.bus = bus
        self.extractor = SignalExtractor(dbc_name, bus)
        
        if not self.extractor.signal_mapping:
            print(f"[WARNING] No target signals found in {dbc_name}")
            print("   Simulator will run but extracted values may be None.")
        
        self.packer = CANPacker(dbc_name)
        
        # Current simulation state
        self.speed = 0.0  # km/h
        self.reverse_gear = 0
        self.illumination = 0
        self.swc_buttons = 0
        self.running = False
        self.last_messages = []  # For display
        
        print(f"[OK] Simulator initialized for {dbc_name}")
        print("[CONTROLS] Keyboard controls:")
        print("   - Press 'w' to increase speed")
        print("   - Press 's' to decrease speed")
        print("   - Press 'r' to toggle reverse gear")
        print("   - Press 'l' to toggle illumination")
        print("   - Press 'b' to cycle SWC buttons")
        print("   - Press 'q' to quit")
    
    def _generate_can_data(self, timestamp_nanos: int):
        """Generate CAN messages using CANPacker."""
        messages = []
        message_info = []
        
        for msg_name, signals in self.extractor.signal_mapping.items():
            try:
                signal_values = {}
                
                # Set speed signals
                if 'vehspeed' in signals:
                    for sig_name in signals['vehspeed']:
                        signal_values[sig_name] = self.speed
                
                # Set gear signals
                if 'reversegear' in signals:
                    for sig_name in signals['reversegear']:
                        signal_values[sig_name] = self.reverse_gear
                
                # Set illumination
                if 'illumination' in signals:
                    for sig_name in signals['illumination']:
                        signal_values[sig_name] = self.illumination
                
                # Set SWC buttons
                if 'swc_buttons' in signals:
                    for sig_name in signals['swc_buttons']:
                        signal_values[sig_name] = self.swc_buttons
                
                # Pack the message
                if signal_values:
                    msg_addr, msg_data, msg_bus = self.packer.make_can_msg(msg_name, self.bus, signal_values)
                    if msg_addr > 0 and len(msg_data) > 0:
                        messages.append((msg_addr, msg_data, msg_bus))
                        message_info.append({
                            'name': msg_name,
                            'addr': msg_addr,
                            'bus': msg_bus,
                            'data': msg_data,
                            'signals': signal_values.copy()
                        })
            except Exception as e:
                # Skip if packing fails
                continue
        
        self.last_messages = message_info
        
        # Return in the format extractor expects
        return [(timestamp_nanos, messages)]
    
    def _print_can_messages(self, can_data):
        """Print CAN messages with nice formatting."""
        # Clear old lines
        for _ in range(self.lines_printed):
            sys.stdout.write(self.CLEAR_LINE + self.CURSOR_UP)
        sys.stdout.flush()
        
        self.lines_printed = 0
        
        if self.last_messages:
            timestamp = can_data[0][0] if can_data and can_data[0] else 0
            
            # Header
            header = f"{self.CYAN}{self.BOLD}╔═══ LAST SENT CAN MESSAGES ═══╗{self.RESET}"
            print(header)
            self.lines_printed += 1
            
            info_line = f"{self.CYAN}║{self.RESET} Count: {self.GREEN}{len(self.last_messages)}{self.RESET} | Timestamp: {self.DIM}{timestamp}{self.RESET} {self.CYAN}║{self.RESET}"
            print(info_line)
            self.lines_printed += 1
            
            print(f"{self.CYAN}╠{'═'*66}╣{self.RESET}")
            self.lines_printed += 1
            
            # Sort messages by type
            speed_msgs = []
            gear_msgs = []
            illum_msgs = []
            swc_msgs = []
            other_msgs = []
            
            for msg_info in self.last_messages:
                signal_types = set()
                if msg_info['signals']:
                    for sig_name in msg_info['signals'].keys():
                        sig_lower = sig_name.lower()
                        if any(x in sig_lower for x in ['speed', 'vehspeed', 'vehicle_speed', 'car_speed', 'hudspeedset']):
                            signal_types.add('speed')
                        if any(x in sig_lower for x in ['gear', 'reverse', 'gearshifter', 'prndl', 'gearsel', 'swigear']):
                            signal_types.add('gear')
                        if any(x in sig_lower for x in ['illum', 'light', 'lighting', 'lights_on', 'alightstat', 'alight']):
                            signal_types.add('illum')
                        if any(x in sig_lower for x in ['swc', 'button', 'steering_wheel', 'cruise', 'btn', 'switch']):
                            signal_types.add('swc')
                
                # Add to categories (can be in multiple)
                has_category = False
                if 'speed' in signal_types:
                    speed_msgs.append(msg_info)
                    has_category = True
                if 'gear' in signal_types:
                    gear_msgs.append(msg_info)
                    has_category = True
                if 'illum' in signal_types:
                    illum_msgs.append(msg_info)
                    has_category = True
                if 'swc' in signal_types:
                    swc_msgs.append(msg_info)
                    has_category = True
                
                if not has_category:
                    other_msgs.append(msg_info)
            
            # Print each category
            all_categorized = [
                (speed_msgs, 'SPEED', self.GREEN),
                (gear_msgs, 'REVERSE GEAR', self.MAGENTA),
                (illum_msgs, 'ILLUMINATION', self.YELLOW),
                (swc_msgs, 'SWC BUTTONS', self.BLUE),
            ]
            
            for msg_list, category, color in all_categorized:
                if msg_list:
                    # Category header
                    cat_header = f"{self.CYAN}║{self.RESET} {color}{self.BOLD}[{category}]{self.RESET} " + " " * 50 + f"{self.CYAN}║{self.RESET}"
                    print(cat_header)
                    self.lines_printed += 1
                    
                    # Messages in category
                    for msg_info in msg_list:
                        data_hex = ' '.join(f'{b:02X}' for b in msg_info['data'])
                        data_part = f"[{data_hex}]"
                        
                        msg_line = f"{self.CYAN}║{self.RESET}   {color}{msg_info['name']:18s}{self.RESET} {self.BOLD}0x{msg_info['addr']:03X}{self.RESET}  Bus:{msg_info['bus']}  {self.DIM}{data_part}{self.RESET}"
                        
                        # Add signal values with colors
                        if msg_info['signals']:
                            colored_sig_parts = []
                            for k, v in msg_info['signals'].items():
                                sig_lower = k.lower()
                                if any(x in sig_lower for x in ['speed', 'vehspeed', 'vehicle_speed', 'car_speed']):
                                    colored_sig_parts.append(f"{self.GREEN}{k}={v:.1f}{self.RESET}")
                                elif any(x in sig_lower for x in ['gear', 'reverse', 'gearshifter', 'prndl']):
                                    colored_sig_parts.append(f"{self.MAGENTA}{k}={v}{self.RESET}")
                                elif any(x in sig_lower for x in ['illum', 'light', 'lighting', 'lights_on']):
                                    colored_sig_parts.append(f"{self.YELLOW}{k}={v}{self.RESET}")
                                elif any(x in sig_lower for x in ['swc', 'button', 'steering_wheel', 'cruise']):
                                    colored_sig_parts.append(f"{self.BLUE}{k}={v}{self.RESET}")
                                else:
                                    colored_sig_parts.append(f"{k}={v}")
                            
                            signals_str = " | " + ", ".join(colored_sig_parts)
                            # Calculate padding (remove ANSI codes first)
                            base_len = len(msg_info['name']) + 10 + len(f"0x{msg_info['addr']:03X}") + 8 + len(data_part)
                            sig_len_no_ansi = len(signals_str.replace('\033[0m', '').replace('\033[92m', '').replace('\033[95m', '').replace('\033[93m', '').replace('\033[94m', '').replace('\033[96m', '').replace('\033[1m', '').replace('\033[2m', ''))
                            padding = max(0, 60 - base_len - sig_len_no_ansi)
                            msg_line += signals_str + " " * padding
                        
                        msg_line += f"{self.CYAN}║{self.RESET}"
                        print(msg_line)
                        self.lines_printed += 1
            
            # Footer
            print(f"{self.CYAN}╚{'═'*66}╝{self.RESET}")
            self.lines_printed += 1
        else:
            print(f"{self.CYAN}╔═══ LAST SENT CAN MESSAGES ═══╗{self.RESET}")
            self.lines_printed += 1
            print(f"{self.CYAN}║{self.RESET} {self.RED}No messages sent{self.RESET} " + " " * 48 + f"{self.CYAN}║{self.RESET}")
            self.lines_printed += 1
            print(f"{self.CYAN}╚{'═'*66}╝{self.RESET}")
            self.lines_printed += 1
        
        sys.stdout.flush()
    
    def run_interactive(self):
        """Run interactive simulation."""
        # Try to get keyboard input working
        try:
            import msvcrt  # Windows
            getch = lambda: msvcrt.getch().decode('utf-8').lower()
        except ImportError:
            try:
                import termios, tty
                def getch():
                    fd = sys.stdin.fileno()
                    old_settings = termios.tcgetattr(fd)
                    try:
                        tty.setraw(sys.stdin.fileno())
                        ch = sys.stdin.read(1)
                    finally:
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                    return ch.lower()
            except ImportError:
                # Fallback
                def getch():
                    return input("Command (w/s/r/l/b/q): ").lower()[0] if True else ''
        
        self.running = True
        print("\n[START] Starting simulation...\n")
        print("Press 'q' to quit\n")
        
        # Clear screen
        sys.stdout.write(self.CLEAR_SCREEN)
        sys.stdout.flush()
        
        iteration = 0
        while self.running:
            iteration += 1
            
            # Generate CAN data
            timestamp = int(time.time() * 1_000_000_000)  # nanoseconds
            can_data = self._generate_can_data(timestamp)
            
            # Move to top
            sys.stdout.write(self.RESET_CURSOR)
            
            # Print header
            print(f"{self.CYAN}{self.BOLD}╔{'═'*68}╗{self.RESET}")
            header_text = f"CAN SIMULATOR - Iteration {iteration} | Timestamp: {timestamp}"
            print(f"{self.CYAN}║{self.RESET} {self.WHITE}{self.BOLD}{header_text.center(66)}{self.RESET} {self.CYAN}║{self.RESET}")
            print(f"{self.CYAN}╚{'═'*68}╝{self.RESET}")
            print()
            
            # Show CAN messages
            self._print_can_messages(can_data)
            print()
            
            # Update extractor and show results
            self.extractor.update(can_data)
            self.extractor.print_extracted(clear_previous=False)
            
            sys.stdout.flush()
            
            # Check for keypress
            try:
                if msvcrt.kbhit():
                    key = getch()
                    if key == 'q':
                        self.running = False
                        break
                    elif key == 'w':
                        self.speed = min(self.speed + 5, 200)
                        print("[UP] Speed increased")
                    elif key == 's':
                        self.speed = max(self.speed - 5, 0)
                        print("[DOWN] Speed decreased")
                    elif key == 'r':
                        self.reverse_gear = 1 - self.reverse_gear
                        print(f"[GEAR] Reverse gear: {'ON' if self.reverse_gear else 'OFF'}")
                    elif key == 'l':
                        self.illumination = 1 - self.illumination
                        print(f"[LIGHT] Illumination: {'ON' if self.illumination else 'OFF'}")
                    elif key == 'b':
                        self.swc_buttons = (self.swc_buttons + 1) % 16
                        print(f"[BTN] SWC Buttons: {self.swc_buttons}")
            except (NameError, AttributeError):
                pass
            
            time.sleep(0.1)  # 10 Hz


def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python can_simulator.py <dbc_file_name>")
        print("\nExample:")
        print("  python can_simulator.py honda_civic_touring_2016_can_generated")
        print("  python can_simulator.py hyundai_i30_2014")
        return
    
    dbc_name = sys.argv[1]
    # Remove .dbc extension if present
    if dbc_name.endswith('.dbc'):
        dbc_name = dbc_name[:-4]
    
    try:
        simulator = CANSimulator(dbc_name, bus=0)
        simulator.run_interactive()
    except KeyboardInterrupt:
        print("\n\n[BYE] Simulation stopped by user")
    except Exception as e:
        print(f"[ERROR] Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
