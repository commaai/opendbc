# Signal Extractor & CAN Simulator

A pair of tools for extracting and testing vehicle signals from CAN bus messages using DBC (Database CAN) files.

## Overview

This project consists of two complementary tools:

1. **Signal Extractor** (`signal_extractor.py`) - Extracts common vehicle signals from CAN messages
2. **CAN Simulator** (`can_simulator.py`) - Generates simulated CAN messages for testing signal extraction

Together, they provide a complete workflow for working with vehicle CAN bus data: generating test messages, extracting signals, and verifying the extraction process.

## Components

### Signal Extractor

The `SignalExtractor` class automatically discovers and extracts four common vehicle signals from CAN messages:
- **Vehspeed** (Vehicle Speed)
- **ReverseGear** (Reverse Gear Indicator)  
- **Illumination** (Lighting Status)
- **SWC_Buttons** (Steering Wheel Control Buttons)

**Key Features:**
- Automatic signal discovery from DBC files
- Flexible pattern matching for signal names
- Real-time signal extraction
- Formatted visual output

### CAN Simulator

The `CANSimulator` class generates simulated CAN messages and uses the `SignalExtractor` to verify extraction:
- Interactive keyboard controls
- Real-time visualization of messages and extracted signals
- Automatic CAN message generation using DBC definitions
- Color-coded output for easy debugging

## Quick Start

### Prerequisites

- Python 3.x
- `opendbc` package (for CAN parsing and DBC file support)
- A DBC file for your vehicle (available in `opendbc/dbc/` directory)

### Installation

Ensure you have the `opendbc` package installed. The tools use:
- `opendbc.can.CANParser` - For parsing CAN messages
- `opendbc.can.CANPacker` - For generating CAN messages
- `opendbc.can.dbc.DBC` - For reading DBC files

### Basic Usage

#### 1. Test Signal Extraction with the Simulator

The easiest way to get started is using the simulator, which demonstrates both components working together:

```bash
python can_simulator.py honda_civic_touring_2016_can_generated
```

This will:
- Initialize the simulator with the specified DBC file
- Generate CAN messages with simulated vehicle signals
- Use the Signal Extractor to extract signals from those messages
- Display both the generated messages and extracted signals in real-time

**Interactive Controls:**
- `w` - Increase speed
- `s` - Decrease speed
- `r` - Toggle reverse gear
- `l` - Toggle illumination
- `b` - Cycle SWC buttons
- `q` - Quit

#### 2. Use Signal Extractor Standalone

Extract signals from your own CAN data:

```python
from signal_extractor import SignalExtractor

# Initialize extractor
extractor = SignalExtractor('honda_civic_touring_2016_can_generated', bus=0)

# Update with CAN data (format: [(timestamp_nanos, [(addr, data, bus), ...])])
can_data = [(1234567890000000000, [(0x123, b'\x01\x02\x03...', 0)])]
extracted = extractor.update(can_data)

# Print formatted output
extractor.print_extracted()
```

#### 3. List Available DBC Files

To see what DBC files are available:

```bash
python signal_extractor.py
```

This will list available DBC files in the `opendbc/dbc/` directory.

## How They Work Together

```
┌─────────────────┐
│  CAN Simulator  │
│                 │
│  Generates CAN  │
│    messages     │
└────────┬────────┘
         │
         │ CAN data
         ▼
┌─────────────────┐
│ Signal Extractor│
│                 │
│  Extracts       │
│   signals       │
└─────────────────┘
```

1. **CAN Simulator** generates CAN messages based on DBC definitions and current simulation state
2. **Signal Extractor** receives the CAN messages and extracts the target signals
3. Both display their results side-by-side for verification

## Example Workflow

### Testing a New DBC File

1. **Start the simulator:**
   ```bash
   python can_simulator.py your_dbc_file_name
   ```

2. **Observe initialization:**
   - The extractor will search for target signals in the DBC file
   - It will display which messages contain which signals
   - If no signals are found, you'll see a warning

3. **Interact with the simulator:**
   - Use keyboard controls to modify simulation state
   - Watch as CAN messages are generated
   - Verify that extracted signals match your inputs

4. **Verify extraction:**
   - Compare the generated CAN messages (top) with extracted signals (bottom)
   - Ensure values match your expectations
   - Test edge cases (speed = 0, max speed, gear changes, etc.)

### Using with Real CAN Data

```python
from signal_extractor import SignalExtractor

# Initialize with your vehicle's DBC file
extractor = SignalExtractor('your_vehicle_dbc', bus=0)

# In your CAN reading loop
while True:
    # Get CAN data from your hardware (panda, socketcan, etc.)
    can_messages = get_can_messages()  # Your function
    
    # Format: [(timestamp_nanos, [(addr, data, bus), ...])]
    can_data = [(int(time.time() * 1e9), can_messages)]
    
    # Extract signals
    extracted = extractor.update(can_data)
    
    # Use extracted signals
    speed = extracted['vehspeed']
    in_reverse = extracted['reversegear']
    lights_on = extracted['illumination']
    button_state = extracted['swc_buttons']
    
    # Process your signals...
```

## Signal Matching

Both tools use the same signal matching patterns. The extractor searches for signals using these name variations:

| Signal Type | Search Patterns |
|------------|----------------|
| **vehspeed** | `vehspeed`, `vehiclespeed`, `vehicle_speed`, `speed`, `car_speed`, `hudspeedset`, `speed_unit` |
| **reversegear** | `reversegear`, `reverse_gear`, `gear`, `gearshifter`, `prndl`, `gearsel`, `swigear` |
| **illumination** | `illumination`, `lights`, `lighting`, `lights_on`, `alightstat`, `alight`, `lightstat` |
| **swc_buttons** | `swc_buttons`, `swc`, `steering_wheel`, `cruise_buttons`, `buttons`, `btn`, `switch`, `cruise`, `steering` |

The matching is case-sensitive and looks for these patterns within signal names in the DBC file.

## Output Examples

### Simulator Output

```
╔════════════════════════════════════════════════════════════════════════╗
║ CAN SIMULATOR - Iteration 42 | Timestamp: 1234567890000000000         ║
╚════════════════════════════════════════════════════════════════════════╝

╔═══ LAST SENT CAN MESSAGES ═══╗
║ Count: 2 | Timestamp: 1234567890000000000 ║
╠══════════════════════════════════════════════════════════════════════╣
║ [SPEED]                                                               ║
║   VEHICLE_SPEED       0x123  Bus:0  [01 02 03 04 05 06 07 08] | vehspeed=45.0
║ [GEAR]                                                                ║
║   GEAR_STATUS        0x456  Bus:0  [09 0A 0B 0C 0D 0E 0F 10] | reversegear=1
╚══════════════════════════════════════════════════════════════════════╝

╔══════════════════════════════════════════════════════════════════╗
║ EXTRACTED SIGNALS                                                 ║
╠══════════════════════════════════════════════════════════════════╣
║ [SPEED]     Vehspeed:                      45.00 km/h            ║
║ [GEAR]      ReverseGear:                    ON (1)               ║
║ [LIGHT]     Illumination:                   OFF (0)              ║
║ [BTN]       SWC_Buttons:                    3                    ║
╚══════════════════════════════════════════════════════════════════╝
```

## Common Use Cases

### 1. Testing DBC File Compatibility

Use the simulator to verify that a DBC file contains the signals you need:

```bash
python can_simulator.py your_dbc_file
```

Check the initialization output to see which signals were found.

### 2. Debugging Signal Extraction

If signals aren't being extracted correctly:
- Run the simulator to see both generated messages and extracted values
- Compare the raw CAN data with the extracted signals
- Verify signal names match the patterns in the DBC file

### 3. Development and Testing

Use the simulator during development to:
- Test signal extraction logic without hardware
- Verify DBC file changes
- Understand signal mappings
- Debug CAN message packing/unpacking

### 4. Integration with Real Hardware

Use the extractor with real CAN data from:
- Panda hardware
- SocketCAN interfaces
- USB CAN adapters
- Any source that provides CAN messages

## File Structure

```
.
├── signal_extractor.py          # Signal extraction tool
├── can_simulator.py              # CAN simulation tool
├── signal_extractor_README.md   # Detailed Signal Extractor docs
├── can_simulator_README.md      # Detailed CAN Simulator docs
└── README_SIGNAL_TOOLS.md       # This file (overview)
```

## Dependencies

### Required
- `opendbc.can.CANParser` - CAN message parsing
- `opendbc.can.CANPacker` - CAN message generation
- `opendbc.can.dbc.DBC` - DBC file reading

### Platform-Specific (for simulator)
- Windows: `msvcrt` (built-in)
- Unix/Linux/macOS: `termios`, `tty` (built-in)

## Troubleshooting

### No Signals Found

If you see `[WARNING] No target signals found in <dbc_name>`:
- Check that the DBC file exists and is valid
- Verify signal names in the DBC file match the search patterns
- Signal matching is case-sensitive - check exact signal names

### Simulator Not Responding to Keys

- On Windows, ensure you're running in a terminal (not IDLE)
- On Unix systems, ensure terminal supports raw input
- Try pressing keys more deliberately (some terminals have input buffering)

### Extracted Values are None

- Ensure CAN messages are being generated/sent
- Verify the DBC file contains the expected signals
- Check that message addresses match between generation and extraction
- Ensure bus numbers match

## Advanced Usage

### Custom Signal Extraction

You can extend the extractor to search for additional signals by modifying the `target_signals` dictionary in `signal_extractor.py`:

```python
target_signals = {
    'vehspeed': [...],
    'reversegear': [...],
    'illumination': [...],
    'swc_buttons': [...],
    'your_signal': ['pattern1', 'pattern2', ...]  # Add your signal
}
```

### Multiple Bus Support

Both tools support specifying a bus number:

```python
extractor = SignalExtractor('dbc_name', bus=1)  # Use bus 1
simulator = CANSimulator('dbc_name', bus=1)
```

## Documentation

For detailed documentation on each component:

- **[Signal Extractor Documentation](signal_extractor_README.md)** - Complete API reference and usage guide
- **[CAN Simulator Documentation](can_simulator_README.md)** - Interactive controls and simulation details

## License

These tools are part of the opendbc project. See the main project LICENSE file for details.

## Contributing

When contributing:
- Test with multiple DBC files
- Verify both tools work together
- Ensure cross-platform compatibility (Windows/Unix)
- Update documentation for new features

## Support

For issues related to:
- **DBC files**: Check the opendbc project documentation
- **CAN parsing**: See `opendbc.can` module documentation
- **These tools**: Check the individual README files or open an issue

---

**Quick Links:**
- [Signal Extractor Details](signal_extractor_README.md)
- [CAN Simulator Details](can_simulator_README.md)
- [OpenDBC Project](https://github.com/commaai/opendbc)

