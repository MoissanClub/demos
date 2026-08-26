# Source provenance

## Preserved local incident code

The complete locally authored code involved in the incident is stored under
`code/` and verified by `MANIFEST.sha256`:

- Standalone sequence entry point and Unitree client/publisher adapter.
- Bounded trajectory, guards, deadline scheduler, and abort release.
- Sport-mode DDS type.
- Asynchronous telemetry recorder and Unitree telemetry conversion.
- RPC cleanup helper.
- Complete standalone-arm unit tests.
- Operating and safety documentation in effect at incident time.

The snapshots are byte-identical to the working-tree files used for the two
incident runs. They intentionally preserve the unsafe `--execute-arm-sdk` path
for forensic analysis. Do not execute them against hardware.

## Unitree SDK dependency

Repository at incident analysis time:

```text
/home/dwei/unitree_sdk2_python
Git revision: 65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5
```

Relevant dependency hashes:

```text
0617124577a1d41b2fd559d8fbba8bb1b6ace43e0471a7dbbb972b51ac155872  unitree_sdk2py/g1/arm/g1_arm_action_client.py
e929f22cb2bc6e65bbbf2aa1f6f57983a14c1937b53bde7b49c85c30af8e73d0  unitree_sdk2py/g1/arm/g1_arm_action_api.py
b3101722e07534c667d4c673677a8a5d57643f307cfdeb11f4761ebc456b18ea  unitree_sdk2py/idl/default.py
```

The installed `unitree_sdk2py` package also supplies DDS channel, LowCmd,
LowState, CRC, and RPC transport implementations imported by the preserved
code. The Git revision above is the durable identity for that dependency tree.

## Unitree XR reference

Repository used to derive the motion-mode command pattern:

```text
/home/dwei/xr_teleoperate
Git revision: 845b25a32f7febedf220e830952a7134897adb9d
```

Relevant reference hash:

```text
978294754867ad3513432a0cfa9ec72f843ce6e90a4f58df106f5231113a1a24  teleop/robot_control/robot_arm.py
```

The XR file is reference material, not code executed by the incident harness.
