import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# complementary filter csv data
'''
t_s,
gyro_x_rad_s,
gyro_y_rad_s,
gyro_z_rad_s,
accel_x_m_s2,
accel_y_m_s2,
accel_z_m_s2,
ekf_roll_deg,
ekf_pitch_deg,
ekf_yaw_deg,
att_roll_deg,
att_pitch_deg,
att_yaw_deg,
truth_roll_deg,
truth_pitch_deg,
truth_yaw_deg,
truth_source,
dt_s
'''

def rmse(estimate, truth):
    return np.sqrt(np.nanmean((estimate-truth)**2))

def max_error(estimate_deg, truth_deg):
    return np.nanmax(np.abs(estimate_deg - truth_deg))

# give arrays of accel only estimated roll and pitch
def accel_only_attitude(df:pd.DataFrame):
    ax = df["accel_x_m_s2"]
    ay = df["accel_y_m_s2"]
    az = df["accel_z_m_s2"]
    
    phi_accel_rad = np.atan2(-ay, -az)
    theta_accel_rad = np.atan2(ax, np.sqrt(ay**2 + az**2))
    
    return phi_accel_rad, theta_accel_rad


def inject_gyro_bias(df, bias_deg_per_s=0.5, axis="gyro_x_rad_s"):
    df_biased = df.copy()
    bias_rad_s = np.radians(bias_deg_per_s)
    df_biased[axis] = df_biased[axis] + bias_rad_s
    return df_biased


# give arrays of gyro only estimated roll and pitch
def gyro_only_attitude(df:pd.DataFrame):
    t = df["t_s"].values
    dt = df["dt_s"].values
    
    # initial conditions
    first_valid_idx = df["truth_roll_deg"].first_valid_index()
    phi_init = np.radians(df["truth_roll_deg"].loc[first_valid_idx])
    theta_init = np.radians(df["truth_pitch_deg"].loc[first_valid_idx])
    
    p = df["gyro_x_rad_s"].values
    q = df["gyro_y_rad_s"].values
    
    phi_gyro_rad = np.zeros(len(t))
    theta_gyro_rad = np.zeros(len(t))
    
    phi_gyro_rad[0] = phi_init
    theta_gyro_rad[0] = theta_init
    
    
    # skipping index 0 because of initialization
    # populate attitude arrays
    for k in range(1, len(t)):
        phi_gyro_rad[k] = phi_gyro_rad[k-1] + p[k]*dt[k]
        theta_gyro_rad[k] = theta_gyro_rad[k-1] + q[k]*dt[k]
        
    return phi_gyro_rad, theta_gyro_rad


def attitude_complimentary_filter(df:pd.DataFrame, alpha: float):
    
    # storing accel only estimates of phi and theta
    phi_accel_rad, theta_accel_rad = accel_only_attitude(df)
    
    
    t = df["t_s"].values
    dt = df["dt_s"].values
    
    p = df["gyro_x_rad_s"].values
    q = df["gyro_y_rad_s"].values
    
    phi_comp_rad = np.zeros(len(t))
    theta_comp_rad = np.zeros(len(t))
    
    
    # initial conditions
    first_valid_idx = df["truth_roll_deg"].first_valid_index()
    phi_init = np.radians(df["truth_roll_deg"].loc[first_valid_idx])
    theta_init = np.radians(df["truth_pitch_deg"].loc[first_valid_idx])
    
    phi_comp_rad[0] = phi_init
    theta_comp_rad[0] = theta_init
    
    for k in range(1, len(t)):
        phi_term = phi_comp_rad[k-1] + p[k]*dt[k]
        theta_term = theta_comp_rad[k-1] + q[k]*dt[k]
        
        phi_comp_rad[k] = alpha*phi_term + (1-alpha)*phi_accel_rad[k]
        theta_comp_rad[k] = alpha*theta_term + (1-alpha)*theta_accel_rad[k]
        
    return phi_comp_rad, theta_comp_rad


# plot gyro only attitude estimates against truth 
def plot_gyro_only_attitude(df: pd.DataFrame, save_path=None):
    
    t = df["t_s"].values
    dt = df["dt_s"].values
    
    phi_gyro_rad, theta_gyro_rad = gyro_only_attitude(df)
    
    # plotting vs ground truth
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    
    # roll subplot axes[0]
    axes[0].plot(t, df["truth_roll_deg"], color="tab:gray", linewidth=1.5,
                 label="Ground truth roll")
    axes[0].plot(t, np.degrees(phi_gyro_rad), color="tab:blue", linewidth=1.5,
                 label="Gyro-only roll estimate")
    axes[0].set_ylabel("Roll [deg]")
    axes[0].grid(True, linestyle="--", alpha=0.6)
    axes[0].legend(loc="best")
    
    
    # pitch subplot axes[1]
    axes[1].plot(t, df["truth_pitch_deg"], color="tab:gray", linewidth=1.5,
                 label="Ground truth pitch")
    axes[1].plot(t, np.degrees(theta_gyro_rad), color="tab:orange", linewidth=1.5,
                 label="Gyro-only pitch estimate")
    axes[1].set_ylabel("Pitch [deg]")
    axes[1].set_xlabel("Time [s]")
    axes[1].grid(True, linestyle="--", alpha=0.6)
    axes[1].legend(loc="best")

    fig.suptitle("Gyro-Only Attitude Estimate vs. Ground Truth")
    fig.text(
        0.5, -0.03,
        "Figure: Roll and pitch estimated by integrating gyroscope rates alone\n"
        "(gyro_x_rad_s, gyro_y_rad_s), compared against ground truth.",
        ha="center", va="top", fontsize=9, wrap=True
    )
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=200, bbox_inches="tight")
    return fig

def plot_accel_only_attitude(df:pd.DataFrame, save_path=None):
    t = df["t_s"].values
    
    phi_accel_rad, theta_accel_rad = accel_only_attitude(df)
    
    # plotting vs ground truth
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    
    # roll subplot axes[0]
    axes[0].plot(t, df["truth_roll_deg"], color="tab:gray", linewidth=1.5,
                    label="Ground truth roll")
    axes[0].plot(t, np.degrees(phi_accel_rad), color="tab:green", linewidth=1.5,
                    label="Accel-only roll estimate")
    axes[0].set_ylabel("Roll [deg]")
    axes[0].grid(True, linestyle="--", alpha=0.6)
    axes[0].legend(loc="best")
    
    
    # pitch subplot axes[1]
    axes[1].plot(t, df["truth_pitch_deg"], color="tab:gray", linewidth=1.5,
                    label="Ground truth pitch")
    axes[1].plot(t, np.degrees(theta_accel_rad), color="tab:red", linewidth=1.5,
                    label="Accel-only pitch estimate")
    axes[1].set_ylabel("Pitch [deg]")
    axes[1].set_xlabel("Time [s]")
    axes[1].grid(True, linestyle="--", alpha=0.6)
    axes[1].legend(loc="best")

    fig.suptitle("Accel-Only Attitude Estimate vs. Ground Truth")
    fig.text(
        0.5, -0.03,
        "Figure: Roll and pitch estimated by accelerometer alone\n"
        "(accel_x, accel_y), compared against ground truth.",
        ha="center", va="top", fontsize=9, wrap=True
    )
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=200, bbox_inches="tight")
    return fig

def build_comparison_table(df, alpha_final):
    truth_roll = df["truth_roll_deg"].values
    truth_pitch = df["truth_pitch_deg"].values

    # Gyro-only
    phi_gyro_rad, theta_gyro_rad = gyro_only_attitude(df)
    phi_gyro_deg = np.degrees(phi_gyro_rad)
    theta_gyro_deg = np.degrees(theta_gyro_rad)

    # Accel-only
    phi_accel_rad, theta_accel_rad = accel_only_attitude(df)
    phi_accel_deg = np.degrees(phi_accel_rad)
    theta_accel_deg = np.degrees(theta_accel_rad)

    # Complementary filter
    phi_comp_rad, theta_comp_rad = attitude_complimentary_filter(df, alpha_final)
    phi_comp_deg = np.degrees(phi_comp_rad)
    theta_comp_deg = np.degrees(theta_comp_rad)

    # EKF (already in the CSV)
    phi_ekf_deg = df["ekf_roll_deg"].values
    theta_ekf_deg = df["ekf_pitch_deg"].values

    estimators = {
        "Gyroscope Only": (phi_gyro_deg, theta_gyro_deg),
        "Accelerometer Only": (phi_accel_deg, theta_accel_deg),
        "Complementary Filter": (phi_comp_deg, theta_comp_deg),
        "ArduPilot EKF": (phi_ekf_deg, theta_ekf_deg),
    }

    print(f"{'Estimator':<22}{'Roll RMSE':>12}{'Pitch RMSE':>12}{'Max Roll Err':>14}{'Max Pitch Err':>15}")
    for name, (phi_deg, theta_deg) in estimators.items():
        roll_rmse = np.sqrt(np.nanmean((phi_deg - truth_roll)**2))
        pitch_rmse = np.sqrt(np.nanmean((theta_deg - truth_pitch)**2))
        max_roll = max_error(phi_deg, truth_roll)
        max_pitch = max_error(theta_deg, truth_pitch)
        print(f"{name:<22}{roll_rmse:>12.3f}{pitch_rmse:>12.3f}{max_roll:>14.3f}{max_pitch:>15.3f}")

    return estimators


def plot_all_estimators(df, estimators, save_path=None):
    t = df["t_s"].values
    truth_roll = df["truth_roll_deg"].values
    truth_pitch = df["truth_pitch_deg"].values

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    colors = {
        "Gyroscope Only": "tab:blue",
        "Accelerometer Only": "tab:orange",
        "Complementary Filter": "tab:green",
        "ArduPilot EKF": "tab:purple",
    }

    axes[0].plot(t, truth_roll, color="black", linewidth=2, label="Ground truth")
    for name, (phi_deg, _) in estimators.items():
        axes[0].plot(t, phi_deg, color=colors[name], linewidth=1, label=name, alpha=0.8)
    axes[0].set_ylabel("Roll [deg]")
    axes[0].legend(loc="best", fontsize=8)
    axes[0].grid(True, linestyle="--", alpha=0.6)

    axes[1].plot(t, truth_pitch, color="black", linewidth=2, label="Ground truth")
    for name, (_, theta_deg) in estimators.items():
        axes[1].plot(t, theta_deg, color=colors[name], linewidth=1, label=name, alpha=0.8)
    axes[1].set_ylabel("Pitch [deg]")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend(loc="best", fontsize=8)
    axes[1].grid(True, linestyle="--", alpha=0.6)

    fig.suptitle("Attitude Estimator Comparison: Gyro / Accel / Complementary / EKF vs. Truth")
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=200, bbox_inches="tight")
    return fig

def main():
    # run necessary code and make plots
    parser = argparse.ArgumentParser(description="question 2 of homework")
    parser.add_argument("--csv_path", default="../HOMEWORK_complementary_filter.csv", help="Path to the input CSV file.")
    parser.add_argument("--outdir", default="./hw_1_plots",help="output dir")
    
    args = parser.parse_args()
    
    df = pd.read_csv(args.csv_path)
    
    save_gyro_only = None
    save_accel_only = None
    if args.outdir:
        os.makedirs(args.outdir, exist_ok=True)
        save_gyro_only = os.path.join(args.outdir, "gyro_only_attitude.png")
        save_accel_only = os.path.join(args.outdir, "accel_only_attitude.png")
        save_compare = os.path.join(args.outdir, "comparison_attitude.png")
        save_bias = os.path.join(args.outdir, "gyro_bias.png")
        
    
    plot_gyro_only_attitude(df, save_gyro_only)
    plot_accel_only_attitude(df, save_accel_only)

    # Gyro-only
    phi_gyro_rad, theta_gyro_rad = gyro_only_attitude(df)
    phi_gyro_deg = np.degrees(phi_gyro_rad)
    theta_gyro_deg = np.degrees(theta_gyro_rad)

    # Accel-only
    phi_accel_rad, theta_accel_rad = accel_only_attitude(df)
    phi_accel_deg = np.degrees(phi_accel_rad)
    theta_accel_deg = np.degrees(theta_accel_rad)

    # Complementary filter
    phi_comp_rad, theta_comp_rad = attitude_complimentary_filter(df, 0.95)
    phi_comp_deg = np.degrees(phi_comp_rad)
    theta_comp_deg = np.degrees(theta_comp_rad)

    # EKF (already in the CSV)
    phi_ekf_deg = df["ekf_roll_deg"].values
    theta_ekf_deg = df["ekf_pitch_deg"].values

    alpha_final = 0.95

    estimators = {
        "Gyroscope Only": (phi_gyro_deg, theta_gyro_deg),
        "Accelerometer Only": (phi_accel_deg, theta_accel_deg),
        "Complementary Filter": (phi_comp_deg, theta_comp_deg),
        "ArduPilot EKF": (phi_ekf_deg, theta_ekf_deg),
    }
    
    df_biased = inject_gyro_bias(df, bias_deg_per_s=0.5, axis="gyro_x_rad_s")

    phi_gyro_biased_rad, theta_gyro_biased_rad = gyro_only_attitude(df_biased)
    phi_comp_biased_rad, theta_comp_biased_rad = attitude_complimentary_filter(df_biased, alpha_final)
    
    plot_gyro_only_attitude(df_biased)

    
    # Problem 2 part C
    # alphas = [0.50, 0.80, 0.95, 0.98, 0.995]
    
    # # actuals for comparison 
    # truth_roll = df["truth_roll_deg"].values
    # truth_pitch = df["truth_pitch_deg"].values
    
    # for a in alphas:
    #     phi_comp_rad, theta_comp_rad = attitude_complimentary_filter(df, a)
    #     phi_comp_deg = np.degrees(phi_comp_rad)
    #     theta_comp_deg = np.degrees(theta_comp_rad)
        
    #     # calculate RMSE 
    #     roll_error = (phi_comp_deg - truth_roll + 180) % 360 - 180
    #     pitch_error = (theta_comp_deg - truth_pitch + 180) % 360 - 180
        
    #     rmse_phi = np.sqrt(np.nanmean(roll_error**2))
    #     rmse_theta = np.sqrt(np.nanmean(pitch_error**2))
        
    #     print(f"alpha={a:.3f}  roll RMSE={rmse_phi:.3f} deg  pitch RMSE={rmse_theta:.3f} deg")


if __name__ == "__main__":
    main()