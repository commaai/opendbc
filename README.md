# opendbc

opendbc is a Python API for your car. Read the speed, steering angle, and more. Send gas, braking, and steering commands.

## Structure
* [`opendbc/dbc/`](opendbc/dbc/) is a repository of [DBC](https://en.wikipedia.org/wiki/CAN_bus#DBC) files
* [`opendbc/can/`](opendbc/can/) is a library for parsing and building CAN messages from DBC files
* [`opendbc/car/`](opendbc/car/) is a high-level library for interfacing with cars using Python

## Quick start

```bash
git clone https://github.com/commaai/opendbc.git

cd opendbc

# Install the dependencies
pip3 install -e .[testing,docs]

# Build
scons -j8

# Run the tests
pytest .

# Run the linter
pre-commit run --all-files
```
