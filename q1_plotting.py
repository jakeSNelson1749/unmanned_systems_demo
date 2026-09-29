# question 1 figure generation and plotting

import sys
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def plot_trajectory(df, save_path=None):
    """North-East (top-down) ground track."""
    fig, ax = plt.subplots(figsize=(7, 7))

    ax.plot(df["east_m"], df["north_m"], color="tab:blue", linewidth=1.5,
            label="Vehicle trajectory")
    ax.scatter(df["east_m"].iloc[0], df["north_m"].iloc[0],
               color="green", marker="o", s=80, zorder=5, label="Start")
    ax.scatter(df["east_m"].iloc[-1], df["north_m"].iloc[-1],
               color="red", marker="X", s=80, zorder=5, label="End")

    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title("Vehicle Ground Track (North-East Plane)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="best")

    fig.text(
        0.5, -0.02,
        "Figure 1: Top-down view of vehicle position in the local North-East\n"
        "navigation frame."
        "Start and end points are marked; down/altitude axis is not shown.",
        ha="center", va="top", fontsize=9, wrap=True
    )
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=200, bbox_inches="tight")
    return fig


def plot_velocity(df, save_path=None):
    """North, East, Down velocity components vs time."""
    fig, ax = plt.subplots(figsize=(9, 5))

    ax.plot(df["t_s"], df["vn_m_s"], color="tab:blue", label="North velocity (vn)")
    ax.plot(df["t_s"], df["ve_m_s"], color="tab:orange", label="East velocity (ve)")
    ax.plot(df["t_s"], df["vd_m_s"], color="tab:green", label="Down velocity (vd)")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Velocity [m/s]")
    ax.set_title("NED Velocity Components vs. Time")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="best")

    fig.text(
        0.5, -0.04,
        "Figure 2: North, East, and Down velocity components (vn_m_s, ve_m_s, vd_m_s)\n"
        "plotted against mission time. Positive down velocity indicates descent.",
        ha="center", va="top", fontsize=9, wrap=True
    )
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=200, bbox_inches="tight")
    return fig


def plot_attitude(df, save_path=None):
    """Roll, pitch, yaw vs time."""
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    labels = ["Roll", "Pitch", "Yaw"]
    cols = ["roll_deg", "pitch_deg", "yaw_deg"]
    colors = ["tab:blue", "tab:orange", "tab:green"]

    for ax, label, col, color in zip(axes, labels, cols, colors):
        ax.plot(df["t_s"], df[col], color=color, linewidth=1.5, label=label)
        ax.set_ylabel(f"{label} [deg]")
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.legend(loc="best", fontsize=8)

    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("Attitude (Roll / Pitch / Yaw) vs. Time")

    fig.text(
        0.5, -0.02,
        "Figure 3: Vehicle attitude (roll, pitch, yaw) over time, in degrees.",
        ha="center", va="top", fontsize=9, wrap=True
    )
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=200, bbox_inches="tight")
    return fig


def main():
    parser = argparse.ArgumentParser(description="Plot nav/EKF log data from CSV.")
    parser.add_argument("--csv_path", default="../HOMEWORK_vehicle_motion.csv", help="Path to the input CSV file.")
    parser.add_argument("--outdir", default="./homework_1_plots",
                         help="Optional directory to save PNGs instead of just showing them.")
    args = parser.parse_args()

    df = pd.read_csv(args.csv_path)

    save_traj = save_vel = save_att = None
    if args.outdir:
        import os
        os.makedirs(args.outdir, exist_ok=True)
        save_traj = os.path.join(args.outdir, "trajectory_ne.png")
        save_vel = os.path.join(args.outdir, "velocity_ned.png")
        save_att = os.path.join(args.outdir, "attitude_rpy.png")

    plot_trajectory(df, save_traj)
    plot_velocity(df, save_vel)
    plot_attitude(df, save_att)

    plt.show()


if __name__ == "__main__":
    main()