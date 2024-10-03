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

# ./test.sh is the all-in-one that will install deps, build, lint, and test
./test.sh
```

[`examples/`](examples/) contains small example programs that can read state from the car and control the steering, gas, and brakes.
[`examples/joystick.py`](examples/joystick.py) allows you to control a car with a joystick.

## Roadmap

This project was pulled out from [openpilot](https://github.com/commaai/openpilot).
We're still figuring out the exact API between openpilot and opendbc, so some of these
may end up going in openpilot.

* Extend support to every car with LKAS + ACC interfaces
* Automatic lateral and longitudinal control/tuning evaluation
* Auto-tuning for [lateral](https://blog.comma.ai/090release/#torqued-an-auto-tuner-for-lateral-control) and longitudinal control
* [Automatic Emergency Braking](https://en.wikipedia.org/wiki/Automated_emergency_braking_system)
* `pip install opendbc`
* 100% type coverage
* 100% line coverage
* Make car ports easier: refactors, tools, tests, and docs
* Expose the state of all supported cars better: https://github.com/commaai/opendbc/issues/1144

Contributions towards anything here is welcome. Join the [Discord](https://discord.comma.ai)!

## FAQ

* **How do I use this?** Depends on what you want to do. [openpilot](https://github.com/commaai/openpilot) is our development target, but you can also use a [panda](https://comma.ai/shop/panda) for basic control or just reading state from your car.
* **Can I add support for my car?** Yes, most car support comes from the community. Join the [Discord](https://discord.comma.ai) and watch this [talk](https://www.youtube.com/watch?v=XxPS5TpTUnI&t=142s&pp=ygUPY29tbWFfY29uIGphc29u) to get started. (We also offer [paid bounties](https://comma.ai/bounties) on car ports.)
* **Which cars are supported?** See the openpilot [supported cars list](https://github.com/commaai/openpilot/blob/master/docs/CARS.md) and `grep` around the codebase.
