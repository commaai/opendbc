Version 0.3.0 (2026-04-22)
========================
* Supported car count: 345 → 397
* Tesla Model 3, Model Y, and Model X (HW3 and HW4) support thanks to sshane, lukasloetkolben, and greatgitsby!
* Rivian R1S and R1T (Gen 1 and Gen 2) support thanks to lukasloetkolben!
* Porsche Macan and Audi Q5 (VW MLB) support thanks to jyoung8607 and Dennis-NL!
* VW MEB (ID.x) architecture support thanks to jyoung8607!
* PSA AEE2010_R3 platform support thanks to elkoled!
* Jeep Cherokee 2019-23 support thanks to jyoung8607!
* Many new Honda / Acura CAN-FD platforms (Accord, CR-V, Pilot, Passport, Odyssey, HR-V, Civic Hybrid, MDX, RDX, TLX) thanks to jyoung8607, mvl-boston, and vanillagorillaa!
* Honda City 2023 and N-Box 2018 support thanks to thalesac and miettal!
* Ford F-150, F-150 Lightning, Mach-E, Ranger, Expedition, Escape 2023-24, and Kuga 2024 support thanks to incognitojam, alan-polk, and hiimisaac!
* Hyundai Nexo 2021 support thanks to sunnyhaibin!
* Kia K7 2017 support thanks to royjr!
* Lexus LS 2018 support thanks to Hacheoy!
* Lexus RC 2023 support thanks to nelsonjchen!
* CANParser and CANPacker rewritten in pure Python — the installed package is pure Python, no compilation required
* Safety code, `isotp.py`/`ccp.py`/`xcp.py`, and `vehicle_model.py` moved into opendbc from panda/openpilot — opendbc is now the self-contained car API package
* `mull` replaced with a faster custom mutation test runner
* Safety hardening: per-brand message-block config, missing RX checks, relay-malfunction config

Version 0.2.1 (2025-02-10)
========================
* Fix missing files making car/ package not importable

Version 0.2.0 (2025-02-10)
========================
* Moved car/ directory from openpilot to opendbc. It comprises the APIs necessary to communicate with 275+ car models
  * opendbc is moving towards being a complete self-contained car API package
  * Soon all opendbc-related tests from openpilot will be migrated as well

Version 0.1.0 (2024-08-01)
========================
* Initial pre-release package with can/ and dbc/ directories
