
# GlobalAgent Navigation — XLeRobot Scripts

Personal working repo for the navigation side of the GlobalAgent project

(Brown/Northeastern, with Sergio Orozco and George Konidaris). Holds the

XLeRobot control scripts developed on the lab's Alienware Linux laptop, so

they're not stuck on one machine.

## Contents

- `base_teleop.py` — minimal keyboard teleop for the differential-drive base

  only (no arms). Talks directly to the Feetech motor bus. First working

  script for this rig; useful as a low-level reference for the base's

  motor IDs and wiring.

- `xlerobot_2wheels_teleop_keyboard.py` — full teleop: both SO101 arms

  (keyboard-driven inverse kinematics) plus the differential-drive base,

  built on the `XLerobot2Wheels` LeRobot robot class. This is the main

  working script for this hardware. Adapted from XLeRobot's

  `4_xlerobot_2wheels_teleop_keyboard.py` example, with the stock

  3-wheel/head-camera version's head-motor code removed, since this rig's

  camera (Orbbec Femto Bolt) is external and stationary, not on a robot-

  mounted pan/tilt head.

- `lerobot_patches/xlerobot_2wheels.py` — a patched copy of

  `lerobot/robots/xlerobot_2wheels/xlerobot_2wheels.py` (see **Required

  setup** and **Bugs fixed** below). This file does not live in this repo's

  own install path — it has to be manually copied into your local `lerobot`

  package source tree.

## Hardware

XLeRobot 2-wheel differential-drive base, dual SO101 arms (Feetech STS3215

servos), controlled from a laptop over USB (wired, not networked/ZMQ).

Confirmed wiring on the lab rig: `/dev/ttyACM0` = left arm only,

`/dev/ttyACM1` = right arm daisy-chained with the two base wheel motors

(IDs 9, 10).

## Required setup

These scripts assume a working `lerobot` install (HuggingFace's LeRobot)

with the XLeRobot fork's custom robot class merged in. If you're setting

this up fresh:

1. Install LeRobot per [HuggingFace's install guide](https://huggingface.co/docs/lerobot/installation),

   editable (`pip install -e .`).

2. Clone [`Vector-Wangel/XLeRobot`](https://github.com/Vector-Wangel/XLeRobot).

3. Copy the robot class folder into your LeRobot install:

cp -r XLeRobot/software/src/robots/xlerobot_2wheels <path-to-lerobot>/src/lerobot/robots/xlerobot_2wheels

4. **Use the patched version from this repo instead of the one you just

   copied** — overwrite it:

cp lerobot_patches/xlerobot_2wheels.py <path-to-lerobot>/src/lerobot/robots/xlerobot_2wheels/xlerobot_2wheels.py

5. Copy the SO101 kinematics helper (needed by the teleop script):

cp XLeRobot/software/src/model/SO101Robot.py <path-to-lerobot>/src/lerobot/model/SO101Robot.py

6. Register `xlerobot_2wheels` for the `lerobot-calibrate` CLI — it isn't

   picked up automatically. Open

   `<path-to-lerobot>/src/lerobot/scripts/lerobot_calibrate.py` and add

   `xlerobot_2wheels,` to the `from lerobot.robots import (...)` tuple,

   alphabetically after `so_follower,`. Without this, `lerobot-calibrate

   --robot.type=xlerobot_2wheels` fails with "invalid choice," even though

   the class itself imports fine everywhere else.

7. Install `pyzmq` and `rerun-sdk` (`pip install pyzmq 'lerobot[viz]'`) —

   both get imported unconditionally by files in this chain even though

   this wired/local setup doesn't use ZMQ networking.

8. Calibrate both arms and the base:

lerobot-calibrate --robot.type=xlerobot_2wheels --robot.port1=/dev/ttyACM0 --robot.port2=/dev/ttyACM1 --robot.id=my_xlerobot_2wheels_lab

  This is an interactive, hands-on process — you physically move each

   joint through its range while the calibrator watches. Take it slow,

   especially on `elbow_flex`: moving it too far in calibration can wrap

   the encoder past its 0/4095 boundary and register a full 360° "range"

   that isn't real.

Calibration output is cached at

`~/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/<id>.json`

and isn't tracked in this repo, since it's specific to one physical robot's

exact motors and homing offsets, not portable code.

## Bugs fixed in `lerobot_patches/xlerobot_2wheels.py`

The stock version copied from XLeRobot's repo has four bugs affecting the

headless 2-wheel variant specifically:

1. **Phantom head motors on `bus1`.** The motor dict hardcoded

   `head_motor_1`/`head_motor_2` (IDs 7, 8) even though this variant has no

   head servos. This made `connect()` fail outright, since it scans for

   motors that don't physically exist.

2. **`configure()` called `bus2.configure_motors()` twice and never called

   it on `bus1`.** This skips a Feetech-specific register fix

   (clearing the `Phase` register's angle-feedback bit) needed for the

   STS3215 to report clean, non-overflowing position values — silently

   affecting the left arm — and the duplicated call on `bus2` produced a

   transient comms timeout during `enable_torque()` on at least one run.

3. **`get_observation()` called `bus1.sync_read(..., self.head_motors)`

   unconditionally.** Once fix #1 correctly makes `self.head_motors` an

   empty list, this crashes with `StopIteration` on every single

   observation read. Guarded with `if self.head_motors else {}`.

4. **Swapped sign convention in `_body_to_wheel_raw()`.** The `x` (forward)

   term was applied with the same sign to both wheels, and `theta`

   (rotation) with opposite signs — backwards from this rig's actual wheel

   mounting. Result: forward/backward keys produced rotation, and rotate

   keys produced forward/backward motion. Fixed by flipping the sign

   convention to match what `base_teleop.py` empirically proved correct.

## Known non-fixes / caveats

- `xlerobot_2wheels_teleop_keyboard.py` also had two behavioral gaps fixed

  directly in this repo's copy (not in the `lerobot_patches` file, since

  they're specific to the example script, not the robot class):

  - The advertised `b: Quit` key was never actually wired into the main

    loop.

  - Quitting called `move_to_zero_position()`, which snaps to a hardcoded

    zero pose and then cuts torque on disconnect — meaning the arm goes

    slack while still away from a supported resting pose. Replaced with a

    `return_to_start_position()` method (per arm) that eases back to

    wherever the arm was when the script connected, before torque drops.

- `_wheel_raw_to_body()` (used only for observation/telemetry, not actual

  motion) may have the same sign-convention issue as fix #4 above — this

  hasn't been confirmed or fixed yet.

