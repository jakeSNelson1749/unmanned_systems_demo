#!/usr/bin/env python3

import time

from unmanned_systems_basics.quadcopter import Quadcopter


vehicle = Quadcopter("udp:127.0.0.1:14551")
vehicle.set_mode("GUIDED")

# Assumes the vehicle is already airborne.
#
# NED convention:
#   +VN = North
#   +VE = East
#   +VD = Down
#
# Therefore VD = -1 m/s is an upward/climb command.
max_speed: float = 10.0
time_duration: float = 5.0
sequence = [
    # duration [s], VN [m/s], VE [m/s], VD [m/s]
    (time_duration/2.0, 0.0, 0.0, 0.0),
    (time_duration, max_speed, 0.0, 0.0),   # North
    (time_duration, -max_speed, 0.0, 0.0),  # South
    (time_duration, 0.0, max_speed, 0.0),   # East
    (time_duration, 0.0, -max_speed, 0.0),  # West
    # (time_duration, 0.0, 0.0, -1.0),  # Climb
    (time_duration/2.0, 0.0, 0.0, 0.0),   # Stop
]

try:
    for duration_s, vn, ve, vd in sequence:
        end_time = time.monotonic() + duration_s

        while time.monotonic() < end_time:
            state = vehicle.update()

            vehicle.command_velocity_ned(
                vn_m_s=vn,
                ve_m_s=ve,
                vd_m_s=vd,
            )

            print(
                f"\r"
                f"cmd=[{vn:4.1f}, {ve:4.1f}, {vd:4.1f}] m/s | "
                f"meas=[{state.vn_m_s:5.2f}, "
                f"{state.ve_m_s:5.2f}, "
                f"{state.vd_m_s:5.2f}] m/s",
                end="",
            )

            time.sleep(0.05)

finally:
    vehicle.stop()
    print("\nVelocity demo complete.")
