# Signal Extractor

A simple signal extractor for CAN messages that extracts common vehicle signals from DBC-defined CAN bus data.

## Overview

The `SignalExtractor` class automatically searches for and extracts four common vehicle signals from CAN messages:
- **Vehspeed** (Vehicle Speed)
- **ReverseGear** (Reverse Gear Indicator)
- **Illumination** (Lighting Status)
- **SWC_Buttons** (Steering Wheel Control Buttons)

## Features

- **Automatic Signal Discovery**: Automatically finds relevant signals in DBC files by matching common signal name patterns
- **Flexible Matching**: Supports multiple signal name variations (e.g., `vehspeed`, `vehicle_speed`, `speed`, etc.)
- **Real-time Extraction**: Updates and extracts signals from live CAN data
- **Visual Display**: Pretty-printed output with color-coded signal values

## Usage

### As a Module

```python
from signal_extractor import SignalExtractor

# Initialize with a DBC file name (without .dbc extension)
extractor = SignalExtractor('honda_civic_touring_2016_can_generated', bus=0)

# Update with CAN data
can_data = [(timestamp_nanos, [(addr, data, bus), ...])]
extracted = extractor.update(can_data)

# Or extract without updating
extracted = extractor.extract()

# Print formatted output
extractor.print_extracted()
```

### As a Script

```bash
python signal_extractor.py <dbc_file_name>
```

Example:
```bash
python signal_extractor.py honda_civic_touring_2016_can_generated
```

If no DBC file is specified, the script will list available DBC files.

## Signal Matching

The extractor searches for signals using these patterns:

| Signal Type | Search Patterns |
|------------|----------------|
| **vehspeed** | `vehspeed`, `vehiclespeed`, `vehicle_speed`, `speed`, `car_speed`, `hudspeedset`, `speed_unit` |
| **reversegear** | `reversegear`, `reverse_gear`, `gear`, `gearshifter`, `prndl`, `gearsel`, `swigear` |
| **illumination** | `illumination`, `lights`, `lighting`, `lights_on`, `alightstat`, `alight`, `lightstat` |
| **swc_buttons** | `swc_buttons`, `swc`, `steering_wheel`, `cruise_buttons`, `buttons`, `btn`, `switch`, `cruise`, `steering` |

## API Reference

### `SignalExtractor(dbc_name: str, bus: int = 0)`

Initialize the signal extractor.

**Parameters:**
- `dbc_name`: Name of the DBC file (without `.dbc` extension)
- `bus`: CAN bus number (default: 0)

**Attributes:**
- `dbc_name`: The DBC file name
- `bus`: The CAN bus number
- `dbc`: The DBC object
- `signal_mapping`: Dictionary mapping message names to their found signals
- `parser`: The CANParser instance (None if no signals found)

### `update(can_data) -> dict`

Update the parser with new CAN data and extract signals.

**Parameters:**
- `can_data`: CAN data in format `[(timestamp_nanos, [(addr, data, bus), ...])]`

**Returns:**
- Dictionary with keys: `vehspeed`, `reversegear`, `illumination`, `swc_buttons`
- Values are extracted signal values or `None` if not found

### `extract() -> dict`

Extract signals from the current parser state without updating.

**Returns:**
- Dictionary with keys: `vehspeed`, `reversegear`, `illumination`, `swc_buttons`
- Values are extracted signal values or `None` if not found

### `print_extracted(extracted=None, clear_previous=False)`

Print extracted signals in a formatted box.

**Parameters:**
- `extracted`: Optional dictionary of extracted values (uses `extract()` if None)
- `clear_previous`: If True, clears previous output lines

## Output Format

The `print_extracted()` method displays signals in a formatted box:

```
╔══════════════════════════════════════════════════════════════════╗
║ EXTRACTED SIGNALS                                                 ║
╠══════════════════════════════════════════════════════════════════╣
║ [SPEED]     Vehspeed:                      45.50 km/h            ║
║ [GEAR]      ReverseGear:                    ON (1)               ║
║ [LIGHT]     Illumination:                   OFF (0)              ║
║ [BTN]       SWC_Buttons:                    5                    ║
╚══════════════════════════════════════════════════════════════════╝
```

## Dependencies

- `opendbc.can.CANParser`: For parsing CAN messages
- `opendbc.can.dbc.DBC`: For reading DBC files
- `opendbc.DBC_PATH`: Path to DBC files directory

## Testing

Use the `can_simulator.py` script to test the signal extractor:

```bash
python can_simulator.py <dbc_file_name>
```

The simulator generates CAN messages and uses the signal extractor to verify extraction works correctly.

## Notes

- If no target signals are found in the DBC file, the extractor will still work but return `None` for all signals
- The extractor takes the first matching signal found in each message (usually the primary signal)
- Signal matching is not case-insensitive
- The extractor automatically discovers which messages contain the target signals during initialization

