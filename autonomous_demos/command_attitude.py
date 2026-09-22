#!/usr/bin/env python3

import time

from unmanned_systems_basics.quadcopter import Quadcopter


vehicle = Quadcopter("udp:127.0.0.1:14551")
vehicle.set_mode("GUIDED")


# ------------------------------------------------------------
# Get fresh telemetry
# ------------------------------------------------------------

print("Waiting for attitude telemetry...")

start = time.monotonic()

while time.monotonic() - start < 1.0:
    state = vehicle.update()
    time.sleep(0.02)

yaw_hold_deg = state.yaw_deg

print(f"Holding yaw at {yaw_hold_deg:.1f} deg")


# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------

time_duration_s = 5.0
desired_angle_deg = 20.0

sequence = [
    # duration, roll, pitch
    (time_duration_s,  desired_angle_deg, 0.0),
    (time_duration_s, -desired_angle_deg, 0.0),
    (time_duration_s, 0.0,  desired_angle_deg),
    (time_duration_s, 0.0, -desired_angle_deg),
]


# ------------------------------------------------------------
# Helper: stop horizontal motion
# ------------------------------------------------------------

def stop_horizontal_motion():

    print("\nStopping horizontal motion...")

    timeout = time.monotonic() + 5.0

    while time.monotonic() < timeout:
        state = vehicle.update()
        vehicle.command_velocity_ned(
            vn_m_s=0.0,
            ve_m_s=0.0,
            vd_m_s=0.0,
            yaw_deg=yaw_hold_deg,
        )

        horizontal_speed = (
            state.vn_m_s ** 2
            + state.ve_m_s ** 2
        ) ** 0.5

        print(
            f"\r"
            f"VN={state.vn_m_s:6.2f} m/s | "
            f"VE={state.ve_m_s:6.2f} m/s | "
            f"speed={horizontal_speed:5.2f} m/s",
            end="",
        )

        if horizontal_speed < 0.15:
            break

        time.sleep(0.05)

    print()


try:

    # Make sure we start nearly stationary.
    stop_horizontal_motion()

    for duration_s, roll_cmd, pitch_cmd in sequence:
        end_time = time.monotonic() + duration_s
        while time.monotonic() < end_time:
            state = vehicle.update()
            vehicle.command_attitude(
                roll_deg=roll_cmd,
                pitch_deg=pitch_cmd,
                yaw_deg=yaw_hold_deg,
                thrust=0.5,
            )

            print(
                f"\r"
                f"CMD R/P = "
                f"[{roll_cmd:5.1f}, {pitch_cmd:5.1f}] deg | "
                f"MEAS R/P = "
                f"[{state.roll_deg:5.1f}, "
                f"{state.pitch_deg:5.1f}] deg | "
                f"Yaw = "
                f"{state.yaw_deg:6.1f}/"
                f"{yaw_hold_deg:6.1f} deg | "
                f"VN/VE = "
                f"[{state.vn_m_s:5.2f}, "
                f"{state.ve_m_s:5.2f}] m/s",
                end="",
            )

            time.sleep(0.02)
            
        stop_horizontal_motion()

finally:

    vehicle.command_velocity_ned(
        vn_m_s=0.0,
        ve_m_s=0.0,
        vd_m_s=0.0,
        yaw_deg=yaw_hold_deg,
    )

    print("\nAttitude demo complete.")