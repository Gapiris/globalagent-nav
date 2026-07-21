"""
base_teleop.py
Minimal keyboard teleop for XLeRobot 2-wheel base only.
No arms or head required.

Controls:
    i - forward
    k - backward  
    u - rotate left
    o - rotate right
    n - speed up
    m - speed down
    b - quit
"""

import sys
import tty
import termios
import numpy as np
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.motors import Motor, MotorNormMode

# --- Configuration ---
PORT = "/dev/ttyACM1"
WHEEL_RADIUS = 0.05   # meters
WHEELBASE = 0.25      # meters

SPEED_LEVELS = [
    {"linear": 0.1, "angular": 30},   # slow
    {"linear": 0.2, "angular": 60},   # medium
    {"linear": 0.3, "angular": 90},   # fast
    {"linear": 0.4, "angular":120},
    {"linear": 0.5, "angular":150},
    {"linear": 0.6, "angular":180},
    {"linear": 0.7, "angular":180}, 
]

KEYS = {
    "forward":      "i",
    "backward":     "k",
    "rotate_left":  "u",
    "rotate_right": "o",
    "speed_up":     "n",
    "speed_down":   "m",
    "quit":         "b",
}


def get_key():
    """Read a single keypress without requiring Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def degps_to_raw(degps: float) -> int:
    """Convert degrees/second to raw motor command."""
    steps_per_deg = 4096.0 / 360.0
    raw = int(round(degps * steps_per_deg))
    return max(-0x8000, min(0x7FFF, raw))


def body_to_wheel_raw(x: float, theta: float) -> dict:
    """
    Convert body-frame velocities to wheel raw commands.
    x     : linear velocity m/s
    theta : rotational velocity deg/s
    """
    theta_rad = theta * (np.pi / 180.0)

    # Differential drive kinematics
    left_speed  = (theta_rad  - x * WHEELBASE / 2) / WHEEL_RADIUS
    right_speed = (theta_rad + x* WHEELBASE / 2) / WHEEL_RADIUS

    # deg/s
    left_degps  = left_speed  * (180.0 / np.pi)
    right_degps = right_speed * (180.0 / np.pi)

    # Scale if over limit
    steps_per_deg = 4096.0 / 360.0
    max_raw = 3000
    raws = [abs(d) * steps_per_deg for d in [left_degps, right_degps]]
    if max(raws) > max_raw:
        scale = max_raw / max(raws)
        left_degps  *= scale
        right_degps *= scale

    return {
        "base_left_wheel":  degps_to_raw(left_degps),
        "base_right_wheel": degps_to_raw(right_degps),
    }


def main():
    print("Connecting to wheel motors...")

    bus = FeetechMotorsBus(
        port=PORT,
        motors={
            "base_left_wheel":  Motor(9,  "sts3215", MotorNormMode.RANGE_M100_100),
            "base_right_wheel": Motor(10, "sts3215", MotorNormMode.RANGE_M100_100),
        },
    )
    bus.connect()
    print("Connected!")

    # Set both wheels to velocity mode
    bus.write("Operating_Mode", "base_left_wheel",  OperatingMode.VELOCITY.value)
    bus.write("Operating_Mode", "base_right_wheel", OperatingMode.VELOCITY.value)
    bus.enable_torque()
    print("Wheels ready.")

    speed_index = 0

    print("\n--- Base Teleop ---")
    print("i/k : forward/backward")
    print("u/o : rotate left/right")
    print("n/m : speed up/down")
    print("b   : quit")
    print(f"Starting at speed level 1 (linear={SPEED_LEVELS[0]['linear']}m/s)\n")

    try:
        while True:
            key = get_key()

            if key == KEYS["quit"]:
                print("\nQuitting...")
                break

            if key == KEYS["speed_up"]:
                speed_index = min(speed_index + 1, len(SPEED_LEVELS) - 1)
                print(f"Speed level {speed_index + 1}: "
 	              f"linear={SPEED_LEVELS[speed_index]['linear']}m/s "
                      f"angular={SPEED_LEVELS[speed_index]['angular']}deg/s")
                continue

            if key == KEYS["speed_down"]:
                speed_index = max(speed_index - 1, 0)
                print(f"Speed level {speed_index + 1}: "
                      f"linear={SPEED_LEVELS[speed_index]['linear']}m/s "
                      f"angular={SPEED_LEVELS[speed_index]['angular']}deg/s")
                continue

            linear  = SPEED_LEVELS[speed_index]["linear"]
            angular = SPEED_LEVELS[speed_index]["angular"]

            x_cmd     = 0.0
            theta_cmd = 0.0

            if key == KEYS["forward"]:
                raw = degps_to_raw(linear * (180.0 / np.pi) / WHEEL_RADIUS)
                wheel_cmds = {"base_left_wheel": -raw, "base_right_wheel": raw}
                print(f"Forward raw={raw}")
            elif key == KEYS["backward"]:
                raw = degps_to_raw(linear * (180.0 / np.pi) / WHEEL_RADIUS)
                wheel_cmds = {"base_left_wheel": raw, "base_right_wheel": -raw}
                print(f"Backward raw={raw}")
            elif key == KEYS["rotate_left"]:
                raw = angular * 10  # scale angular deg/s directly to raw
                raw = int(min(raw, 3000))
                wheel_cmds = {"base_left_wheel": raw, "base_right_wheel": raw}
                print(f"Rotate left raw={raw}")
            elif key == KEYS["rotate_right"]:
                raw = angular * 10
                raw = int(min(raw, 3000))
                wheel_cmds = {"base_left_wheel": -raw, "base_right_wheel": -raw}
                print(f"Rotate right raw={raw}")
            elif key == " ":
                wheel_cmds = {"base_left_wheel": 0, "base_right_wheel": 0}
                print("Stop")
            else:
                continue

            bus.sync_write("Goal_Velocity", wheel_cmds)

    finally:
        # Always stop motors on exit
        print("Stopping motors...")
        bus.sync_write(
            "Goal_Velocity",
            {"base_left_wheel": 0, "base_right_wheel": 0},
        )
        bus.disconnect()
        print("Done.")


if __name__ == "__main__":
    main()
