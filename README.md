## DBC file basics

A DBC file encodes, in a humanly readable way, the information needed to understand a vehicle's CAN bus traffic. A vehicle might have multiple CAN buses and every CAN bus is represented by its own dbc file.
Wondering what's the DBC file format? [Here](http://www.socialledge.com/sjsu/index.php?title=DBC_Format) and [Here](https://github.com/stefanhoelzl/CANpy/blob/master/docs/DBC_Specification.md) a couple of good overviews.

## How to start reverse engineering cars

[opendbc](https://github.com/commaai/opendbc) is integrated with [cabana](https://github.com/commaai/openpilot/tree/master/tools/cabana).

Use [panda](https://github.com/commaai/panda) to connect your car to a computer.

## How to use reverse engineered DBC
To create custom CAN simulations or send reverse engineered signals back to the car you can use [CANdevStudio](https://github.com/GENIVI/CANdevStudio) project.

## DBC file preprocessor

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
