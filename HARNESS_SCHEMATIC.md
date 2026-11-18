# Ford F-150 2026 (TRON) - Harness Schematic POC

## Gateway Module Intercept Point
To bypass the standard OBD-II firewall, the harness must intercept the CAN bus directly at the Image Processing Module A (IPMA) or the Gateway Module (GWM).

## Pinout Mapping (RJ45 to Ford CAN-FD)
- **Pin 1 (CAN-H):** Ford IPMA Connector C1 / Pin 14
- **Pin 2 (CAN-L):** Ford IPMA Connector C1 / Pin 15
- **Pin 3 (GND):** Chassis Ground (Standard Ford black/green trace)
- **Pin 4 (12V Power):** Switched 12V from fuse box (Micro3 fuse tap)

*Note: 120-ohm termination resistor is required on the harness side if intercepting mid-bus.*
