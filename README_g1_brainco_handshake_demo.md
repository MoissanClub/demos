# BrainCo Tactile Handshake Demo for Unitree G1

This README documents `g1_brainco_handshake_demo.py`, a standalone BrainCo SDK demo for the Unitree G1 PC2. The script reads the tactile sensor on one BrainCo hand and commands that same hand to perform a simple handshake behavior.

## Behavior

The demo runs a small state machine:

```text
open_wait:
  Keep the hand fully open while no touch is detected.

closing:
  When touch/contact is detected, close the hand slowly.

hold:
  Stop closing when either:
    1. tactile value reaches the stop threshold, or
    2. commanded close reaches max-close.
  Say the configured greeting once when entering this state.
  Reopen after hold-duration even if contact continues.

release:
  When touch is released for release-seconds, enter the releasing state.
  Command the hand open and wait for measured-open confirmation before sending
  the arm release action. A bounded timeout prevents waiting forever if motor
  status is unavailable.
  The next handshake is blocked until touch is clearly released again.

optional arm action:
  With --enable-arm, trigger the Unitree high-level "shake hand"
  arm action when touch first starts the closing state.
```

Default closing limit:

```text
0    = fully open
1000 = fully closed
500  = half closed
```

The default `--max-close 500` therefore means the hand will not close beyond 50% of the command range. A value of 750 can be selected explicitly after conservative testing.

## Files

Recommended location on PC2:

```text
~/demos/g1_brainco_handshake_demo.py
~/demos/handshake_state.py
~/demos/handshake_speaker.py
~/demos/handshake_config.json
~/demos/telemetry_probe.py
~/bin/g1_fix_serial_permissions.sh
```

The permission helper script is optional but recommended because the BrainCo FTDI serial ports may not be writable by your user after reboot.

## Read-only telemetry discovery

`telemetry_probe.py` records timestamped raw BrainCo tactile/motor samples and
Unitree G1 `LowState` messages as JSON Lines. It never sends hand positions, arm
actions, or low-level robot commands. Enabling BrainCo touch sensors is a device
write, so that setup is disabled unless `--enable-touch-sensors` is specified.

Probe the right hand for 30 seconds:

```bash
python ~/demos/telemetry_probe.py --brainco --right --duration 30
```

Probe Unitree state without controlling the robot:

```bash
python ~/demos/telemetry_probe.py --unitree --network-interface eth0 --duration 30
```

Both sources can be selected together. Output is written under `telemetry/` and
ends with a summary containing observed sample rates, read latencies, and errors.

### Record telemetry during a handshake

The handshake controller can record its existing BrainCo reads together with
read-only Unitree G1 joint and IMU state. BrainCo motor status is sampled once
per control-loop iteration while recording. JSONL file writes run in a bounded
background queue and cannot block the safety/control loop.

```bash
python ~/demos/g1_brainco_handshake_demo.py \
  --right \
  --enable-arm \
  --record-telemetry \
  --telemetry-output ~/demos/telemetry/trajectories/test_run \
  --arm-network-interface eth0
```

Recorded streams include `brainco.touch`, `brainco.motor`,
`unitree.lowstate`, `controller.decision`, `controller.command`, and
`controller.event`. Idle `open_wait` data is never written. Each transition out
of `open_wait` creates a new trajectory JSONL file. That file continues through
`closing`, `hold`, and `releasing`, includes the final transition back to
`open_wait`, and is then atomically finalized. Exiting during an active
handshake finalizes that trajectory as `aborted`; exiting while idle may produce
zero trajectories.

Every finalized trajectory is uploaded after safe robot cleanup and local file
closure to the configured private dataset repository. The default is equivalent
to:

```bash
--upload-trajectories --hf-dataset-repo davidwei79/g1-handshake-data
```

The dataset repository is created as private if necessary. An upload failure
never removes the local trajectory files. Use `--no-upload-trajectories` for a
local-only run.

## Hardware and port mapping

On this G1 setup, the BrainCo module is connected through one USB-C cable, but Linux sees one FTDI FT4232H device exposing four serial interfaces:

```text
FTDI if00 -> /dev/ttyUSB0
FTDI if01 -> /dev/ttyUSB1
FTDI if02 -> /dev/ttyUSB2
FTDI if03 -> /dev/ttyUSB3
```

The working hand mapping discovered by Modbus scan is:

```text
left hand:
  slave ID: 126 / 0x7e
  FTDI interface: if02
  stable port:
    /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if02-port0

right hand:
  slave ID: 127 / 0x7f
  FTDI interface: if01
  stable port:
    /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if01-port0
```

Use `/dev/serial/by-id/...` instead of `/dev/ttyUSB*` where possible, because `/dev/ttyUSB*` numbering can change after reboot.

## Safety notes

Before moving the hand or arm:

1. Keep the G1 body stable.
2. Make sure the arm, hand, fingers, face area, people, hard objects, and cables are clear.
3. Start with `--dry-run`.
4. Start with a conservative `--max-close`, such as `500`.
5. Start with slow closing, such as `--step 10 --period 0.25`.
6. Test hand-only motion before adding `--enable-arm`.
7. Be ready to press `Ctrl-C`.

The script tries to reopen the hand and release the arm on `Ctrl-C`, but do not rely on software alone as the only safety mechanism.

## Important: do not run this with `launch_robot.sh`

Do not run this script while the BrainCo ROS launch is running:

```bash
./launch/launch_robot.sh
# or
~/bin/g1_brainco_launcher.sh robot
```

Reason: `launch_robot.sh` starts `stark_node`, which opens the same BrainCo Modbus serial ports. Only one process should own a given hand serial port at a time.

For this standalone demo, stop the ROS hand node first.

## Install on PC2

Copy the script to the G1 PC2:

```bash
mkdir -p ~/demos
cp g1_brainco_handshake_demo.py ~/demos/
cp handshake_state.py handshake_speaker.py handshake_config.json ~/demos/
chmod +x ~/demos/g1_brainco_handshake_demo.py
```

Make sure the BrainCo SDK is available:

```bash
conda activate g1brainco

cd ~
git clone https://github.com/BrainCoTech/brainco-hand-sdk.git

cd ~/brainco-hand-sdk/python
pip install -r requirements.txt
pip install bc-stark-sdk
```

If you already cloned the SDK earlier, you do not need to clone it again.

## Fix serial permissions

Run the permission helper before testing:

```bash
~/bin/g1_fix_serial_permissions.sh
```

For a persistent setup:

```bash
~/bin/g1_fix_serial_permissions.sh --permanent
sudo reboot
```

After reboot, confirm that your user is in the `dialout` group:

```bash
groups
```

You should see:

```text
dialout
```

## First test: dry run, no hand motion

Use dry-run first. This reads tactile values and prints state transitions, but does not move the hand.

Left hand:

```bash
conda activate g1brainco

python ~/demos/g1_brainco_handshake_demo.py \
  --left \
  --dry-run \
  --duration 30
```

Right hand:

```bash
conda activate g1brainco

python ~/demos/g1_brainco_handshake_demo.py \
  --right \
  --dry-run \
  --duration 30
```

While it is running, touch the fingertips and palm contact areas. Watch the `touch=` value.

Example output:

```text
open_wait armed    touch=   0.00 close_cmd=   0 | thumb=0 index=0 middle=0 ring=0 pinky=0
closing   disarmed touch=  35.00 close_cmd=  20 | thumb=5 index=35 middle=0 ring=0 pinky=0
hold      disarmed touch= 110.00 close_cmd= 220 | thumb=20 index=110 middle=70 ring=0 pinky=0
```

## First motion test: conservative

After dry-run looks reasonable, start with a lower close limit:

```bash
python ~/demos/g1_brainco_handshake_demo.py \
  --left \
  --start-threshold 20 \
  --stop-threshold 80 \
  --release-threshold 10 \
  --max-close 500 \
  --step 10 \
  --period 0.25
```

This means:

```text
start closing when touch >= 20
stop closing when touch >= 80
reopen when touch stays below 10 for 0.7 seconds
never close past 500 / 1000
increase close command by 10 every 0.25 seconds
```

## 3/4-close handshake test

Once the conservative test behaves safely:

```bash
python ~/demos/g1_brainco_handshake_demo.py \
  --left \
  --start-threshold 20 \
  --stop-threshold 80 \
  --release-threshold 10 \
  --max-close 750 \
  --step 20 \
  --period 0.20
```

For the right hand:

```bash
python ~/demos/g1_brainco_handshake_demo.py \
  --right \
  --start-threshold 20 \
  --stop-threshold 80 \
  --release-threshold 10 \
  --max-close 750 \
  --step 20 \
  --period 0.20
```

## Tuning thresholds

The tactile numbers are hardware/firmware dependent. Tune them on your hand.

Recommended tuning workflow:

```bash
python ~/demos/g1_brainco_handshake_demo.py \
  --left \
  --dry-run \
  --duration 60
```

Measure three ranges:

```text
no touch:
  observed background/noise value

light touch:
  value when a person first touches the hand

firm handshake:
  value at a comfortable stopping pressure
```

Then choose:

```text
release-threshold:
  slightly above no-touch noise

start-threshold:
  above release-threshold, near light touch

stop-threshold:
  near comfortable firm contact
```

Example:

```text
no touch:       0-5
light touch:    25-40
firm handshake: 90-130
```

Use:

```bash
--release-threshold 10
--start-threshold 25
--stop-threshold 100
```

## Command reference

```text
--config PATH
  Load behavior settings from this JSON file.
  Default:
    handshake_config.json next to the demo script

--left
  Use the left hand.
  Default left port:
    /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if02-port0
  Default left slave:
    126 / 0x7e

--right
  Use the right hand.
  Default right port:
    /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if01-port0
  Default right slave:
    127 / 0x7f

--port PATH
  Override the serial port.

--slave-id ID
  Override the Modbus slave ID.
  Accepts decimal or hex, for example:
    126
    0x7e

--baud BAUD
  Modbus baudrate.
  Default:
    460800

--start-threshold VALUE
  Touch value that starts closing from open.
  Default:
    50

--stop-threshold VALUE
  Touch value that stops closing.
  Default:
    250

--release-threshold VALUE
  If touch remains below this value for release-seconds, the hand opens.
  Default:
    20

--release-seconds SECONDS
  Release debounce time. While closing or holding, confirmed release starts
  the hand-first release sequence and prevents another activation until touch
  is clearly released again.
  Default:
    0.7

--hold-duration SECONDS
  Maximum time to remain in hold before entering the hand-first release sequence.
  Default:
    5.0

--max-close VALUE
  Maximum close command.
  Range:
    0 to 1000
  Default:
    500

--step VALUE
  Close-command increment per control loop.
  Smaller = slower and safer.
  Default:
    50

--period SECONDS
  Control-loop period.
  Larger = slower.
  Default:
    0.10

--open-repeat SECONDS
  Repeat the open command at this interval while idle.
  Default:
    1.0

--sensor-timeout SECONDS
  Abort through safe cleanup if a tactile read or hand command takes longer
  than this interval. Motor-status timeouts are treated as unavailable status.
  Default:
    1.0

--open-position-threshold VALUE
  The arm is not released until every measured finger position is at or below
  this value, unless open-confirm-timeout expires.
  Default:
    100

--open-confirm-timeout SECONDS
  Maximum time to wait for measured-open confirmation before releasing the arm.
  Default:
    2.0

--thumb-scale SCALE
  Scale thumb and thumb_aux closing relative to other fingers.
  Example:
    --thumb-scale 0.7
  Default:
    1.0

--dry-run
  Read sensors and print decisions, but do not command hand or arm movement.

--duration SECONDS
  Run for a fixed duration.
  0 means run until Ctrl-C.
  Default:
    0

--quiet
  Print less frequently.

--ignore-touch-type-check
  Continue when the SDK hardware type does not advertise tactile support.
  Use only after independently confirming that tactile reads work.

--enable-arm
  Trigger a Unitree high-level arm action when touch first starts the closing state.
  Default:
    disabled

--arm-network-interface IFACE
  DDS network interface shared by the Unitree arm and speaker services, for example eth0.
  If omitted, Unitree SDK auto-detection is used.

--arm-action NAME
  Unitree arm action to run when touch starts closing.
  Default:
    shake hand

--arm-release-action NAME
  Unitree arm action to run after the hand is measured open, and on safe cleanup.
  Default:
    release arm

--arm-release-delay SECONDS
  Deprecated compatibility option. Independent delayed arm release is disabled
  because it can lower the arm while the hand is still gripping.
```

## Greeting configuration

The speaker greeting is loaded from `handshake_config.json`:

```json
{
  "greeting_phrase": "很高兴认识你",
  "speaker_id": 0
}
```

The phrase is spoken once when the controller enters `hold`, whether hold was
caused by the tactile stop threshold or the maximum close command. Speech runs
in a background thread so it does not block hand control. If the Unitree audio
service is unavailable, the script logs a warning and continues the handshake
without speech.

## Useful examples

### Left hand dry-run

```bash
python ~/demos/g1_brainco_handshake_demo.py --left --dry-run --duration 30
```

### Right hand dry-run

```bash
python ~/demos/g1_brainco_handshake_demo.py --right --dry-run --duration 30
```

### Very slow, safer motion

```bash
python ~/demos/g1_brainco_handshake_demo.py \
  --left \
  --max-close 500 \
  --step 5 \
  --period 0.30 \
  --start-threshold 20 \
  --stop-threshold 80 \
  --release-threshold 10
```

### 3/4-close handshake

```bash
python ~/demos/g1_brainco_handshake_demo.py \
  --left \
  --max-close 750 \
  --step 20 \
  --period 0.20 \
  --start-threshold 20 \
  --stop-threshold 80 \
  --release-threshold 10
```

### Hand close plus Unitree arm shake

```bash
python ~/demos/g1_brainco_handshake_demo.py \
  --right \
  --enable-arm \
  --arm-network-interface eth0 \
  --max-close 500 \
  --step 10 \
  --period 0.25 \
  --start-threshold 20 \
  --stop-threshold 80 \
  --release-threshold 10
```

When touch reaches `--start-threshold`, the script enters `closing` and triggers Unitree's built-in `shake hand` arm action. On release or hold timeout, it enters `releasing`, commands the fingers open, and polls measured finger positions. It sends `release arm` only after all fingers are at or below `--open-position-threshold`, or after `--open-confirm-timeout` if confirmation is unavailable. The old independent `--arm-release-delay` behavior is disabled.

### Use explicit port and slave ID

```bash
python ~/demos/g1_brainco_handshake_demo.py \
  --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if02-port0 \
  --slave-id 0x7e \
  --max-close 750
```

## Troubleshooting

### `Permission denied`

Example:

```text
Failed to open Modbus: Permission denied
```

Fix:

```bash
~/bin/g1_fix_serial_permissions.sh
```

Permanent fix:

```bash
~/bin/g1_fix_serial_permissions.sh --permanent
sudo reboot
```

### `Port does not exist`

Check device names:

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB*
```

Expected names include:

```text
usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if01-port0
usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if02-port0
```

If the FTDI serial number changed, run with explicit `--port`, or set environment variables before using helper scripts.

### No touch values change

Possible causes:

1. Wrong hand selected.
2. Wrong serial port.
3. Wrong slave ID.
4. Hand is not a tactile-capable model.
5. Another process is holding the serial port.
6. Touch thresholds are too high.

Try:

```bash
python ~/brainco-hand-sdk/python/demo/hand_monitor.py \
  -m /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if02-port0 460800 126 \
  touch \
  --duration 30
```

### Hand does not move

Check:

1. You are not using `--dry-run`.
2. Serial permissions are fixed.
3. The selected port/slave ID responds.
4. `launch_robot.sh` is not running.
5. The hand has power.

Try a smaller motion first:

```bash
python ~/demos/g1_brainco_handshake_demo.py \
  --left \
  --max-close 300 \
  --step 5 \
  --period 0.30
```

### Hand closes too hard

Reduce these:

```bash
--max-close
--step
```

Increase this:

```bash
--period
```

Lower this if you want it to stop earlier:

```bash
--stop-threshold
```

Example safer command:

```bash
python ~/demos/g1_brainco_handshake_demo.py \
  --left \
  --max-close 450 \
  --step 5 \
  --period 0.30 \
  --stop-threshold 50
```

### Hand closes immediately even when no one is touching it

Your `start-threshold` is too low, or the tactile readings have an offset.

Run dry-run:

```bash
python ~/demos/g1_brainco_handshake_demo.py --left --dry-run --duration 30
```

Then set:

```text
start-threshold > no-touch value
release-threshold slightly above no-touch value
```

Example:

```bash
--release-threshold 20
--start-threshold 40
```

## Exit behavior

Press:

```text
Ctrl-C
```

Normal completion, Ctrl-C cancellation, and handled runtime errors all pass through the same best-effort cleanup path. It commands the hand open, waits for measured-open confirmation up to `--open-confirm-timeout`, then releases the arm and closes Modbus. A second interrupt, loss of communication, or hardware failure can still prevent cleanup. Always test with a safe hand pose and keep the robot clear of people and objects during development.

## Notes on ROS integration

This script is standalone and talks directly to the BrainCo SDK over Modbus. It does not require `launch_robot.sh`, `stark_node`, or the ROS transition node.

A full ROS version would require one of the following:

1. Modify `stark_node` to publish tactile data and subscribe to handshake behavior commands.
2. Write a separate ROS node that owns the serial port and publishes both tactile state and hand commands.
3. Add tactile feedback to the existing BrainCo G1 state machine.

For a quick tactile handshake proof-of-concept, the standalone SDK route is simpler and avoids fighting over the serial port.
