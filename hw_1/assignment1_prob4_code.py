"""
Problem 4(b): Quadcopter Outer-Loop Position Control

Recipe:
- Set desired position
- Calculate position error
- Feed error into proportional controller
- Send velocity command to quadcopter
- Repeat
- Log data and plot results
"""

import time
import numpy as np
import matplotlib.pyplot as plt

from unmanned_systems_basics.quadcopter import Quadcopter

def run_multiple_positions(
    vehicle,
    positions_list,
    kp,
    velocity_limit,
    position_tolerance,
    control_rate,
    duration
):
    """
    Sequentially navigate through multiple desired positions
    and plot the complete North-East trajectory.
    """

    dt = 1.0 / control_rate
    start_time = time.time()

    # Log the complete trajectory
    trajectory_N = []
    trajectory_E = []
    trajectory_D = []
    trajectory_time = []

    for i, position_desired in enumerate(positions_list):

        print(f"\nMoving to Position {i + 1}: {position_desired}")

        target_reached = False

        while time.time() - start_time < duration:

            # Read current vehicle state
            state = vehicle.update()

            position_current = np.array([
                state.north_m,
                state.east_m,
                state.down_m
            ])

            # Calculate position error and velocity command
            position_error, velocity_cmd = position_controller(
                position_desired,
                position_current,
                kp,
                velocity_limit
            )

            # Send velocity command
            vehicle.command_velocity_ned(
                vn_m_s=float(velocity_cmd[0]),
                ve_m_s=float(velocity_cmd[1]),
                vd_m_s=float(velocity_cmd[2])
            )

            # Log position
            trajectory_N.append(position_current[0])
            trajectory_E.append(position_current[1])
            trajectory_D.append(position_current[2])
            trajectory_time.append(time.time() - start_time)

            # Check if target has been reached
            if np.linalg.norm(position_error) < position_tolerance:

                print(f"Position {i + 1} reached!")
                print(f"Elapsed time: {trajectory_time[-1]:.2f} s")

                target_reached = True
                break

            time.sleep(dt)

        if not target_reached:
            print(f"Timeout before reaching Position {i + 1}.")
            break

    # Convert logged data to NumPy arrays
    trajectory_N = np.array(trajectory_N)
    trajectory_E = np.array(trajectory_E)

    # --------------------------------------------------
    # Plot the complete North-East trajectory
    # --------------------------------------------------

    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot the actual vehicle trajectory
    ax.plot(
        trajectory_N,
        trajectory_E,
        color="tab:blue",
        linewidth=2,
        label="Actual Vehicle Trajectory"
    )

    # Mark the starting position
    ax.scatter(
        trajectory_N[0],
        trajectory_E[0],
        color="green",
        marker="o",
        s=100,
        label="Starting Position",
        zorder=5
    )

    # Plot and label each desired position
    for i, position_desired in enumerate(positions_list):

        north = position_desired[0]
        east = position_desired[1]

        ax.scatter(
            north,
            east,
            color="red",
            marker="x",
            s=150,
            linewidths=3,
            zorder=5
        )

        ax.annotate(
            f"Position {i + 1}\n(N={north:.1f}, E={east:.1f})",
            (north, east),
            xytext=(8, 8),
            textcoords="offset points",
            fontsize=10
        )

    # Add labels and formatting
    ax.set_xlabel("North Position [m]")
    ax.set_ylabel("East Position [m]")

    ax.set_title("Complete North-East Quadcopter Trajectory")

    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend()
    ax.set_aspect("equal", adjustable="datalim")

    plt.tight_layout()

    fig.savefig(
        "./hw_1_plots/figure4_North_East_trajectory.png",
        dpi=300,
        bbox_inches="tight"
    )

    return {
        "time": np.array(trajectory_time),
        "position_N": trajectory_N,
        "position_E": trajectory_E,
        "position_D": np.array(trajectory_D)
}


def position_controller(
    position_desired,
    position_current,
    kp,
    velocity_limiter
):
    # Position error in NED coordinates
    position_error = compute_error(position_current, position_desired)

    # Proportional position-to-velocity controller
    
    # @ symbol does matrix multiplication 
    # I replaced the gain matrix with a 3x1 for simplier reading 
    velocity_cmd = kp * position_error

    # Clip velocity command (velocity saturation)
    velocity_cmd = np.clip(
        velocity_cmd,
        -velocity_limiter,
        velocity_limiter
    )

    return position_error, velocity_cmd


def compute_error(actual, desired):
    return desired - actual

# Coordinate frame: NED
# North, East, Down

vehicle = Quadcopter("udp:127.0.0.1:14551")

# Desired target position [North, East, Down], meters
#position_desired = np.array([10.0, 12.0, -5.0])
position_desired = np.array([0.0, 0.0, -10.0])

positions_list = (np.array([-5.0,5.0, -14]),
                  np.array([0.0,0.0, -8]),
                  np.array([10.0,15.0, -16])
)

# Proportional gains [1/s]
kp = np.array([0.8, 0.8, 0.8])

# Maximum commanded velocities [m/s]
velocity_limit = np.array([5.0, 5.0, 2.0])

# Simulation settings
duration = 120.0
control_rate = 10.0
dt = 1.0 / control_rate

# Stop when the vehicle is sufficiently close to the target
position_tolerance = 0.2

log = {
    "time": [],
    "position_N": [],
    "position_E": [],
    "position_D": [],
    "error_N": [],
    "error_E": [],
    "error_D": [],
    "velocity_N": [],
    "velocity_E": [],
    "velocity_D": [],
    "velocity_cmd_N": [],
    "velocity_cmd_E": [],
    "velocity_cmd_D": [],
}

start_time = time.time()

trajectory_log = run_multiple_positions(
    vehicle=vehicle,
    positions_list=positions_list,
    kp=kp,
    velocity_limit=velocity_limit,
    position_tolerance=position_tolerance,
    control_rate=control_rate,
    duration=duration
)


# try:
#     while time.time() - start_time < duration:

#         # Read updated vehicle state
#         state = vehicle.update()

#         position_current = np.array([
#             state.north_m,
#             state.east_m,
#             state.down_m
#         ])

#         velocity_current = np.array([
#             state.vn_m_s,
#             state.ve_m_s,
#             state.vd_m_s
#         ])

#         position_error, velocity_cmd = position_controller(
#             position_desired,
#             position_current,
#             kp,
#             velocity_limit
#         )

#         vehicle.command_velocity_ned(
#             vn_m_s=float(velocity_cmd[0]),
#             ve_m_s=float(velocity_cmd[1]),
#             vd_m_s=float(velocity_cmd[2])
#         )

#         elapsed_time = time.time() - start_time

#         log["time"].append(elapsed_time)

#         log["position_N"].append(position_current[0])
#         log["position_E"].append(position_current[1])
#         log["position_D"].append(position_current[2])

#         log["error_N"].append(position_error[0])
#         log["error_E"].append(position_error[1])
#         log["error_D"].append(position_error[2])

#         log["velocity_N"].append(velocity_current[0])
#         log["velocity_E"].append(velocity_current[1])
#         log["velocity_D"].append(velocity_current[2])

#         log["velocity_cmd_N"].append(velocity_cmd[0])
#         log["velocity_cmd_E"].append(velocity_cmd[1])
#         log["velocity_cmd_D"].append(velocity_cmd[2])

#         # Check if target position has been reached

#         if np.linalg.norm(position_error) < position_tolerance:
#             print("Target position reached.")
#             print(f"Elapsed Time: {log['time'][-1]:.2f} seconds")   # fixed: single quotes inside f-string

#             err_N = np.array(log["error_N"])
#             err_E = np.array(log["error_E"])
#             err_D = np.array(log["error_D"])
#             vcmd_N = np.array(log["velocity_cmd_N"])
#             vcmd_E = np.array(log["velocity_cmd_E"])
#             vcmd_D = np.array(log["velocity_cmd_D"])

#             def axis_overshoot(err):
                
#                 # calculate overshoot
#                 if len(err) == 0 or err[0] == 0:
#                     return 0.0
#                 sign0 = np.sign(err[0])
#                 overshoot_series = sign0 * (-err)          # = sign0 * (actual - desired)
#                 positive_part = overshoot_series[overshoot_series > 0]
#                 return float(np.max(positive_part)) if positive_part.size else 0.0

#             def axis_oscillates(err):
#                 # does it change signs more than once?
#                 s = np.sign(err)
#                 s = s[s != 0]
#                 if s.size < 2:
#                     return False
#                 sign_changes = np.sum(np.diff(s) != 0)
#                 return sign_changes >= 2

#             overshoot_N = axis_overshoot(err_N)
#             overshoot_E = axis_overshoot(err_E)
#             overshoot_D = axis_overshoot(err_D)
#             max_overshoot = max(overshoot_N, overshoot_E, overshoot_D)

#             oscillation_observed = (
#                 axis_oscillates(err_N) or axis_oscillates(err_E) or axis_oscillates(err_D)
#             )

#             err_norm = np.sqrt(err_N**2 + err_E**2 + err_D**2)
#             n_tail = min(5, len(err_norm))
#             steady_state_error = float(np.mean(err_norm[-n_tail:]))

#             all_cmd_vel = np.concatenate([vcmd_N, vcmd_E, vcmd_D])
#             max_commanded_velocity = float(np.max(np.abs(all_cmd_vel)))

#             sat_tol = 1e-3
#             saturation_N = np.any(np.isclose(np.abs(vcmd_N), velocity_limit[0], atol=sat_tol))
#             saturation_E = np.any(np.isclose(np.abs(vcmd_E), velocity_limit[1], atol=sat_tol))
#             saturation_D = np.any(np.isclose(np.abs(vcmd_D), velocity_limit[2], atol=sat_tol))
#             saturation_active = bool(saturation_N or saturation_E or saturation_D)

#             print(f"Maximum overshoot (m): {max_overshoot:.3f}  "
#                   f"[N={overshoot_N:.3f}, E={overshoot_E:.3f}, D={overshoot_D:.3f}]")
#             print(f"Oscillation observed?: {oscillation_observed}")
#             print(f"Steady-state error (m): {steady_state_error:.3f}")
#             print(f"Maximum commanded velocity (m/s): {max_commanded_velocity:.3f}")
#             print(f"Velocity saturation active?: {saturation_active}  "
#                   f"[N={saturation_N}, E={saturation_E}, D={saturation_D}]")

#             break

#         time.sleep(dt)
# finally:
#     pass

# for key in log:
#     log[key] = np.array(log[key])

# t = log["time"]

# # --------------------------------------------------
# # Figure 1: NED position versus time, with desired position
# # --------------------------------------------------

# fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# position_data = [
#     ("position_N", position_desired[0], "North Position [m]", "North"),
#     ("position_E", position_desired[1], "East Position [m]", "East"),
#     ("position_D", position_desired[2], "Down Position [m]", "Down"),
# ]

# for ax, (key, desired, ylabel, label) in zip(axes, position_data):
#     ax.plot(t, log[key], color="tab:blue", label=f"Measured {label} position")
#     ax.axhline(desired, color="tab:red", linestyle="--", label=f"Desired {label} position")
#     ax.set_ylabel(ylabel)
#     ax.grid(True, linestyle="--", alpha=0.6)
#     ax.legend(loc="best")

# axes[-1].set_xlabel("Time [s]")
# fig.suptitle("NED Position vs. Time")
# fig.text(
#     0.5, -0.02,
#     "Figure 1: Measured North, East, and Down position of the quadcopter over time,\n"
#     "compared against the fixed desired position commanded to the outer-loop position\n"
#     "controller. All axes are in the local NED navigation frame, in meters.",
#     ha="center", va="top", fontsize=9, wrap=True
# )
# fig.tight_layout()
# fig.savefig("./hw_1_plots/figure1_position_vs_time.png", dpi=300, bbox_inches="tight")


# # --------------------------------------------------
# # Figure 2: Position error versus time
# # --------------------------------------------------

# fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# error_data = [
#     ("error_N", "North Position Error [m]", "North"),
#     ("error_E", "East Position Error [m]", "East"),
#     ("error_D", "Down Position Error [m]", "Down"),
# ]

# for ax, (key, ylabel, label) in zip(axes, error_data):
#     ax.plot(t, log[key], color="tab:orange", label=f"{label} position error")
#     ax.axhline(0, color="black", linestyle="--", linewidth=1, label="Zero error")
#     ax.set_ylabel(ylabel)
#     ax.grid(True, linestyle="--", alpha=0.6)
#     ax.legend(loc="best")

# axes[-1].set_xlabel("Time [s]")
# fig.suptitle("NED Position Error vs. Time")
# fig.text(
#     0.5, -0.02,
#     "Figure 2: Position error (desired minus measured position) in the North, East,\n"
#     "and Down axes over time, in meters. The error should decay toward zero as the\n"
#     "outer-loop position controller drives the quadcopter to the desired position.",
#     ha="center", va="top", fontsize=9, wrap=True
# )
# fig.tight_layout()
# fig.savefig("./hw_1_plots/figure2_position_error_vs_time.png", dpi=300, bbox_inches="tight")


# # --------------------------------------------------
# # Figure 3: Commanded versus measured velocity
# # --------------------------------------------------

# fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# velocity_data = [
#     ("velocity_N", "velocity_cmd_N", "North Velocity [m/s]", "North"),
#     ("velocity_E", "velocity_cmd_E", "East Velocity [m/s]", "East"),
#     ("velocity_D", "velocity_cmd_D", "Down Velocity [m/s]", "Down"),
# ]

# for ax, (measured, commanded, ylabel, label) in zip(axes, velocity_data):
#     ax.plot(t, log[measured], color="tab:green", label=f"Measured {label} velocity")
#     ax.plot(t, log[commanded], color="tab:purple", linestyle="--", label=f"Commanded {label} velocity")
#     ax.set_ylabel(ylabel)
#     ax.grid(True, linestyle="--", alpha=0.6)
#     ax.legend(loc="best")

# axes[-1].set_xlabel("Time [s]")
# fig.suptitle("Commanded and Measured NED Velocity vs. Time")
# fig.text(
#     0.5, -0.02,
#     "Figure 3: Commanded NED velocity (output of the proportional position controller,\n"
#     "subject to velocity saturation) versus the quadcopter's measured NED velocity,\n"
#     "in meters per second. Close tracking indicates the inner-loop flight controller\n"
#     "is successfully achieving the outer loop's velocity commands.",
#     ha="center", va="top", fontsize=9, wrap=True
# )
# fig.tight_layout()
# fig.savefig("./hw_1_plots/figure3_velocity_vs_time.png", dpi=300, bbox_inches="tight")