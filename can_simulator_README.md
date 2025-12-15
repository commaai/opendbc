# CAN Simulator

An interactive CAN bus simulator for testing signal extraction from DBC-defined CAN messages.

## Overview

The `CANSimulator` class generates simulated CAN messages and uses the `SignalExtractor` to verify that signals are correctly extracted. It provides an interactive terminal interface with keyboard controls to modify simulation parameters in real-time.

## Features

- **Interactive Controls**: Keyboard-based controls to modify simulation state
- **Real-time Visualization**: Live display of generated CAN messages and extracted signals
- **Signal Generation**: Automatically generates CAN messages using `CANPacker` based on DBC definitions
- **Visual Feedback**: Color-coded output showing message types, addresses, and signal values
- **Cross-platform**: Works on Windows (using `msvcrt`) and Unix-like systems (using `termios`)

## Usage

### As a Script

```bash
python can_simulator.py <dbc_file_name>
```

Examples:
```bash
python can_simulator.py honda_civic_touring_2016_can_generated
python can_simulator.py hyundai_i30_2014
```

The DBC file name should be provided without the `.dbc` extension.

### As a Module

```python
from can_simulator import CANSimulator

# Initialize simulator
simulator = CANSimulator('honda_civic_touring_2016_can_generated', bus=0)

# Run interactive simulation
simulator.run_interactive()
```

## Interactive Controls

While the simulator is running, use these keyboard controls:

| Key | Action |
|-----|--------|
| **w** | Increase speed by 5 km/h (max 200 km/h) |
| **s** | Decrease speed by 5 km/h (min 0 km/h) |
| **r** | Toggle reverse gear (0 ↔ 1) |
| **l** | Toggle illumination/lights (0 ↔ 1) |
| **b** | Cycle SWC buttons (0-15) |
| **q** | Quit the simulator |

## Simulated Signals

The simulator generates CAN messages containing:

1. **Vehspeed** (Vehicle Speed)
   - Range: 0-200 km/h
   - Increment: 5 km/h per 'w' press
   - Decrement: 5 km/h per 's' press

2. **ReverseGear** (Reverse Gear Indicator)
   - Values: 0 (OFF) or 1 (ON)
   - Toggle with 'r' key

3. **Illumination** (Lighting Status)
   - Values: 0 (OFF) or 1 (ON)
   - Toggle with 'l' key

4. **SWC_Buttons** (Steering Wheel Control Buttons)
   - Range: 0-15
   - Cycles through values with 'b' key

## Display Output

The simulator displays:

1. **Header**: Current iteration and timestamp
2. **CAN Messages**: List of generated messages showing:
   - Message name
   - CAN address (hex)
   - Bus number
   - Raw data bytes (hex)
   - Signal values with color coding
3. **Extracted Signals**: Formatted box showing extracted values (from `SignalExtractor`)

### Message Categories

Messages are color-coded by signal type:
- 🟢 **GREEN**: Speed-related messages
- 🟣 **MAGENTA**: Reverse gear messages
- 🟡 **YELLOW**: Illumination messages
- 🔵 **BLUE**: SWC button messages

## API Reference

### `CANSimulator(dbc_name: str, bus: int = 0)`

Initialize the CAN simulator.

**Parameters:**
- `dbc_name`: Name of the DBC file (without `.dbc` extension)
- `bus`: CAN bus number (default: 0)

**Attributes:**
- `dbc_name`: The DBC file name
- `bus`: The CAN bus number
- `extractor`: SignalExtractor instance for verifying extraction
- `packer`: CANPacker instance for generating messages
- `speed`: Current speed value (km/h)
- `reverse_gear`: Current reverse gear state (0 or 1)
- `illumination`: Current illumination state (0 or 1)
- `swc_buttons`: Current SWC button value (0-15)
- `running`: Simulation running flag

### `_generate_can_data(timestamp_nanos: int) -> list`

Generate CAN messages based on current simulation state.

**Parameters:**
- `timestamp_nanos`: Timestamp in nanoseconds

**Returns:**
- CAN data in format: `[(timestamp_nanos, [(addr, data, bus), ...])]`

### `run_interactive()`

Start the interactive simulation loop.

- Updates at 10 Hz (0.1 second intervals)
- Clears and redraws the display each iteration
- Responds to keyboard input non-blockingly
- Stops when 'q' is pressed or KeyboardInterrupt occurs

## Example Output

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

## Dependencies

- `signal_extractor.SignalExtractor`: For extracting and verifying signals
- `opendbc.can.CANPacker`: For packing CAN messages
- `msvcrt` (Windows) or `termios` (Unix): For keyboard input handling

## Platform Support

- **Windows**: Uses `msvcrt.getch()` for non-blocking keyboard input
- **Unix/Linux/macOS**: Uses `termios` and `tty` for raw terminal input
- **Fallback**: If neither is available, falls back to `input()` (less ideal)

## Notes

- The simulator runs at 10 Hz (updates every 0.1 seconds)
- If no target signals are found in the DBC file, the simulator will still run but extracted values may be `None`
- The display uses ANSI escape codes for colors and cursor positioning
- Press 'q' or Ctrl+C to exit gracefully
- The simulator automatically discovers which messages contain target signals from the DBC file

## Testing Workflow

1. Start the simulator with a DBC file:
   ```bash
   python can_simulator.py <dbc_file_name>
   ```

2. Use keyboard controls to modify simulation state

3. Observe:
   - Generated CAN messages in the top section
   - Extracted signals in the bottom section
   - Verify that extracted signals match the simulation state

4. This is useful for:
   - Testing signal extraction logic
   - Verifying DBC file correctness
   - Debugging CAN message packing
   - Understanding signal mappings

