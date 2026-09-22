#!/usr/bin/env python3
"""
ArduPilot DataFlash parser for the Unmanned Systems course.

This script extracts the data students need for:

1. Vehicle Motion and Sensor Investigation
   - NED position: PN, PE, PD
   - NED velocity: VN, VE, VD
   - roll, pitch, yaw
   - body angular rates: p, q, r
   - body accelerations: ax, ay, az

2. Complementary Filter Development / EKF Comparison
   - accelerometer measurements
   - gyroscope measurements
   - ArduPilot EKF roll/pitch/yaw
   - ATT roll/pitch/yaw
   - SITL truth roll/pitch/yaw when a SIM message is available

The parser creates:
    <log>_vehicle_motion.csv
    <log>_complementary_filter.csv
    <log>_student_all.csv
    <log>_README.txt

Optionally, it can also save the raw extracted message tables.

Example:
    python parse_student_flight_data.py data/00000005.BIN

Notes
-----
* ArduPilot DataFlash message availability depends on firmware and logging
  configuration. The code is intentionally defensive and will use available
  messages where possible.
* XKF1 is treated as the primary EKF/navigation-state source.
* ATT is also exported because it is useful for comparison and debugging.
* SIM is used as simulation ground truth only when it exists in the log.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np
import pandas as pd
from pymavlink import DFReader


# ---------------------------------------------------------------------------
# Message / field configuration
# ---------------------------------------------------------------------------

DEFAULT_TYPES: List[str] = [
    "IMU",
    "IMU2",
    "ATT",
    "XKF1",
    "GPS",
    "BARO",
    "MAG",
    "SIM",
    "SIM2",
    "CTUN",
    "MODE",
]

# Common aliases seen across ArduPilot versions / message definitions.
FIELD_ALIASES = {
    "roll": ["Roll", "roll"],
    "pitch": ["Pitch", "pitch"],
    "yaw": ["Yaw", "yaw"],
    "vn": ["VN", "VNorth", "VelN"],
    "ve": ["VE", "VEast", "VelE"],
    "vd": ["VD", "VDown", "VelD"],
    "pn": ["PN", "PosN", "N"],
    "pe": ["PE", "PosE", "E"],
    "pd": ["PD", "PosD", "D"],
    "gyr_x": ["GyrX", "GyroX", "GX"],
    "gyr_y": ["GyrY", "GyroY", "GY"],
    "gyr_z": ["GyrZ", "GyroZ", "GZ"],
    "acc_x": ["AccX", "AccelX", "AX"],
    "acc_y": ["AccY", "AccelY", "AY"],
    "acc_z": ["AccZ", "AccelZ", "AZ"],
}


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------

class FlightParser:
    def __init__(self, log_path: str | Path) -> None:
        self.log_path = str(log_path)
        self.binary_log = DFReader.DFReader_binary(filename=self.log_path)

        present = sorted(
            fmt.name for fmt in self.binary_log.formats.values()
            if getattr(fmt, "name", None)
        )
        print("Message types present:")
        print(", ".join(present))

        try:
            counts = {
                self.binary_log.id_to_name[i]: c
                for i, c in enumerate(self.binary_log.counts)
                if c > 0 and i in self.binary_log.id_to_name
            }
            print("\nCounts by type (non-zero):")
            print(counts)
        except Exception:
            pass

        self.binary_log.rewind()

    def get_desired_data(
        self,
        types: Sequence[str],
    ) -> Dict[str, pd.DataFrame]:
        """
        Extract selected DataFlash messages into DataFrames.

        IMPORTANT:
        A single common TimeUS origin is used for every message type so that
        tables can later be synchronized correctly.
        """
        if not types:
            raise ValueError("types cannot be empty")

        requested = set(types)
        rows: Dict[str, List[dict]] = {t: [] for t in types}
        global_time_us: List[float] = []

        self.binary_log.rewind()

        while True:
            msg = self.binary_log.recv_msg()
            if msg is None:
                break

            msg_type = msg.get_type()
            if msg_type not in requested:
                continue

            d = msg.to_dict()

            time_us = getattr(msg, "TimeUS", None)
            if time_us is None:
                time_us = d.get("TimeUS")

            d["TimeUS"] = time_us
            d["_timestamp"] = getattr(msg, "_timestamp", None)
            rows[msg_type].append(d)

            if time_us is not None and np.isfinite(time_us):
                global_time_us.append(float(time_us))

        if not global_time_us:
            raise RuntimeError(
                "No valid TimeUS values were found in the requested message types."
            )

        t0_us = min(global_time_us)

        dfs: Dict[str, pd.DataFrame] = {}
        for msg_type in types:
            df = pd.DataFrame(rows[msg_type])

            if df.empty:
                dfs[msg_type] = df
                continue

            if "TimeUS" not in df.columns:
                dfs[msg_type] = df
                continue

            df = df.dropna(subset=["TimeUS"]).copy()
            df["TimeUS"] = pd.to_numeric(df["TimeUS"], errors="coerce")
            df = df.dropna(subset=["TimeUS"])
            df = df.sort_values("TimeUS").drop_duplicates(
                subset=["TimeUS"], keep="first"
            )
            df["t"] = (df["TimeUS"] - t0_us) * 1e-6
            dfs[msg_type] = df.reset_index(drop=True)

        return dfs


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------

def first_existing_column(
    df: pd.DataFrame,
    candidates: Iterable[str],
) -> Optional[str]:
    """Return the first candidate column that exists in df."""
    for name in candidates:
        if name in df.columns:
            return name
    return None


def add_selected_column(
    output: pd.DataFrame,
    source: pd.DataFrame,
    candidates: Sequence[str],
    output_name: str,
) -> pd.DataFrame:
    """
    Add one renamed source column to a table containing t.

    This helper assumes output and source are already on the same row/time
    basis. It is mainly used after interpolation / merging.
    """
    col = first_existing_column(source, candidates)
    if col is not None:
        output[output_name] = pd.to_numeric(source[col], errors="coerce")
    return output


def interpolate_column(
    source: pd.DataFrame,
    target_t: np.ndarray,
    candidates: Sequence[str],
) -> np.ndarray:
    """
    Interpolate a scalar column onto target_t.

    NaN is returned when the source or requested field is unavailable.
    """
    if source is None or source.empty or "t" not in source.columns:
        return np.full(target_t.shape, np.nan, dtype=float)

    col = first_existing_column(source, candidates)
    if col is None:
        return np.full(target_t.shape, np.nan, dtype=float)

    t = pd.to_numeric(source["t"], errors="coerce").to_numpy(dtype=float)
    y = pd.to_numeric(source[col], errors="coerce").to_numpy(dtype=float)

    valid = np.isfinite(t) & np.isfinite(y)
    if valid.sum() == 0:
        return np.full(target_t.shape, np.nan, dtype=float)

    t = t[valid]
    y = y[valid]

    order = np.argsort(t)
    t = t[order]
    y = y[order]

    t_unique, unique_idx = np.unique(t, return_index=True)
    y_unique = y[unique_idx]

    if len(t_unique) == 1:
        return np.full(target_t.shape, y_unique[0], dtype=float)

    # Do not extrapolate outside the available source interval.
    result = np.interp(target_t, t_unique, y_unique)
    result[(target_t < t_unique[0]) | (target_t > t_unique[-1])] = np.nan
    return result


def interpolate_angle_deg(
    source: pd.DataFrame,
    target_t: np.ndarray,
    candidates: Sequence[str],
) -> np.ndarray:
    """
    Interpolate angle data in degrees while respecting +/-180 wrapping.
    """
    if source is None or source.empty:
        return np.full(target_t.shape, np.nan, dtype=float)

    col = first_existing_column(source, candidates)
    if col is None or "t" not in source.columns:
        return np.full(target_t.shape, np.nan, dtype=float)

    t = pd.to_numeric(source["t"], errors="coerce").to_numpy(dtype=float)
    a_deg = pd.to_numeric(source[col], errors="coerce").to_numpy(dtype=float)
    valid = np.isfinite(t) & np.isfinite(a_deg)

    if valid.sum() == 0:
        return np.full(target_t.shape, np.nan, dtype=float)

    t = t[valid]
    a_rad = np.unwrap(np.deg2rad(a_deg[valid]))

    order = np.argsort(t)
    t = t[order]
    a_rad = a_rad[order]

    t_unique, unique_idx = np.unique(t, return_index=True)
    a_rad = a_rad[unique_idx]

    if len(t_unique) == 1:
        out = np.full(target_t.shape, np.rad2deg(a_rad[0]), dtype=float)
    else:
        out = np.rad2deg(np.interp(target_t, t_unique, a_rad))
        out[(target_t < t_unique[0]) | (target_t > t_unique[-1])] = np.nan

    # Wrap back to [-180, 180)
    out = (out + 180.0) % 360.0 - 180.0
    return out


def choose_imu(dfs: Dict[str, pd.DataFrame]) -> pd.DataFrame:
    """Use IMU if present; otherwise fall back to IMU2."""
    imu = dfs.get("IMU", pd.DataFrame())
    if not imu.empty:
        return imu
    return dfs.get("IMU2", pd.DataFrame())


def choose_truth_attitude_source(
    dfs: Dict[str, pd.DataFrame],
) -> tuple[Optional[str], pd.DataFrame]:
    """
    Use SITL SIM/SIM2 as ground truth if present.

    Returns (message_name, dataframe). If neither is present, returns
    (None, empty DataFrame).
    """
    for name in ("SIM", "SIM2"):
        df = dfs.get(name, pd.DataFrame())
        if not df.empty:
            required = [
                first_existing_column(df, FIELD_ALIASES["roll"]),
                first_existing_column(df, FIELD_ALIASES["pitch"]),
            ]
            if all(c is not None for c in required):
                return name, df

    return None, pd.DataFrame()


# ---------------------------------------------------------------------------
# Student datasets
# ---------------------------------------------------------------------------

def build_complementary_filter_dataset(
    dfs: Dict[str, pd.DataFrame],
) -> pd.DataFrame:
    """
    Build the dataset used to implement the complementary filter.

    The IMU timestamps are preserved so students get the native inertial
    sampling sequence instead of a low-rate resampled table.
    """
    imu = choose_imu(dfs)
    if imu.empty:
        raise RuntimeError(
            "No IMU/IMU2 data found. Enable IMU logging before generating "
            "the student dataset."
        )

    required_imu = {
        "gyro_x_rad_s": FIELD_ALIASES["gyr_x"],
        "gyro_y_rad_s": FIELD_ALIASES["gyr_y"],
        "gyro_z_rad_s": FIELD_ALIASES["gyr_z"],
        "accel_x_m_s2": FIELD_ALIASES["acc_x"],
        "accel_y_m_s2": FIELD_ALIASES["acc_y"],
        "accel_z_m_s2": FIELD_ALIASES["acc_z"],
    }

    out = pd.DataFrame()
    out["t_s"] = pd.to_numeric(imu["t"], errors="coerce")

    for out_name, aliases in required_imu.items():
        col = first_existing_column(imu, aliases)
        if col is None:
            print(f"WARNING: IMU field for {out_name} was not found.")
            out[out_name] = np.nan
        else:
            out[out_name] = pd.to_numeric(imu[col], errors="coerce")

    target_t = out["t_s"].to_numpy(dtype=float)

    # XKF1: primary ArduPilot EKF estimate
    xkf1 = dfs.get("XKF1", pd.DataFrame())
    if not xkf1.empty:
        out["ekf_roll_deg"] = interpolate_angle_deg(
            xkf1, target_t, FIELD_ALIASES["roll"]
        )
        out["ekf_pitch_deg"] = interpolate_angle_deg(
            xkf1, target_t, FIELD_ALIASES["pitch"]
        )
        out["ekf_yaw_deg"] = interpolate_angle_deg(
            xkf1, target_t, FIELD_ALIASES["yaw"]
        )
    else:
        out["ekf_roll_deg"] = np.nan
        out["ekf_pitch_deg"] = np.nan
        out["ekf_yaw_deg"] = np.nan

    # ATT: convenient autopilot attitude output
    att = dfs.get("ATT", pd.DataFrame())
    if not att.empty:
        out["att_roll_deg"] = interpolate_angle_deg(
            att, target_t, FIELD_ALIASES["roll"]
        )
        out["att_pitch_deg"] = interpolate_angle_deg(
            att, target_t, FIELD_ALIASES["pitch"]
        )
        out["att_yaw_deg"] = interpolate_angle_deg(
            att, target_t, FIELD_ALIASES["yaw"]
        )

    # SITL truth if available
    truth_name, truth = choose_truth_attitude_source(dfs)
    if truth_name is not None:
        out["truth_roll_deg"] = interpolate_angle_deg(
            truth, target_t, FIELD_ALIASES["roll"]
        )
        out["truth_pitch_deg"] = interpolate_angle_deg(
            truth, target_t, FIELD_ALIASES["pitch"]
        )
        out["truth_yaw_deg"] = interpolate_angle_deg(
            truth, target_t, FIELD_ALIASES["yaw"]
        )
        out["truth_source"] = truth_name
    else:
        out["truth_roll_deg"] = np.nan
        out["truth_pitch_deg"] = np.nan
        out["truth_yaw_deg"] = np.nan
        out["truth_source"] = "NOT_AVAILABLE"

    # Helpful time-step column for student filter implementation.
    out["dt_s"] = out["t_s"].diff()
    if len(out) > 1:
        median_dt = out["dt_s"].dropna().median()
        out.loc[out.index[0], "dt_s"] = median_dt

    return out


def build_vehicle_motion_dataset(
    dfs: Dict[str, pd.DataFrame],
    rate_hz: float = 20.0,
) -> pd.DataFrame:
    """
    Build a clean, uniformly sampled dataset for vehicle-motion analysis.

    The navigation timeline comes from the overlap between available XKF1,
    ATT, and IMU data. Position/velocity primarily come from XKF1.
    """
    if rate_hz <= 0:
        raise ValueError("rate_hz must be > 0")

    xkf1 = dfs.get("XKF1", pd.DataFrame())
    att = dfs.get("ATT", pd.DataFrame())
    imu = choose_imu(dfs)

    candidates = [df for df in (xkf1, att, imu) if not df.empty and "t" in df]
    if not candidates:
        raise RuntimeError("No XKF1, ATT, or IMU time data available.")

    # Use overlap, not union, so the exported rows are well populated.
    starts = [float(df["t"].min()) for df in candidates]
    ends = [float(df["t"].max()) for df in candidates]

    start_t = max(starts)
    end_t = min(ends)

    if end_t <= start_t:
        # Fallback to the XKF1 interval if sources do not overlap.
        if not xkf1.empty:
            start_t = float(xkf1["t"].min())
            end_t = float(xkf1["t"].max())
        else:
            start_t = min(starts)
            end_t = max(ends)

    dt = 1.0 / rate_hz
    target_t = np.arange(start_t, end_t + 0.5 * dt, dt)

    out = pd.DataFrame({"t_s": target_t})

    # Navigation position and velocity
    for out_name, aliases in [
        ("north_m", FIELD_ALIASES["pn"]),
        ("east_m", FIELD_ALIASES["pe"]),
        ("down_m", FIELD_ALIASES["pd"]),
        ("vn_m_s", FIELD_ALIASES["vn"]),
        ("ve_m_s", FIELD_ALIASES["ve"]),
        ("vd_m_s", FIELD_ALIASES["vd"]),
    ]:
        out[out_name] = interpolate_column(xkf1, target_t, aliases)

    # Prefer ATT for general attitude plotting; also preserve XKF1 attitude.
    for out_name, aliases in [
        ("roll_deg", FIELD_ALIASES["roll"]),
        ("pitch_deg", FIELD_ALIASES["pitch"]),
        ("yaw_deg", FIELD_ALIASES["yaw"]),
    ]:
        out[out_name] = interpolate_angle_deg(att, target_t, aliases)

    for out_name, aliases in [
        ("ekf_roll_deg", FIELD_ALIASES["roll"]),
        ("ekf_pitch_deg", FIELD_ALIASES["pitch"]),
        ("ekf_yaw_deg", FIELD_ALIASES["yaw"]),
    ]:
        out[out_name] = interpolate_angle_deg(xkf1, target_t, aliases)

    # IMU
    for out_name, aliases in [
        ("gyro_x_rad_s", FIELD_ALIASES["gyr_x"]),
        ("gyro_y_rad_s", FIELD_ALIASES["gyr_y"]),
        ("gyro_z_rad_s", FIELD_ALIASES["gyr_z"]),
        ("accel_x_m_s2", FIELD_ALIASES["acc_x"]),
        ("accel_y_m_s2", FIELD_ALIASES["acc_y"]),
        ("accel_z_m_s2", FIELD_ALIASES["acc_z"]),
    ]:
        out[out_name] = interpolate_column(imu, target_t, aliases)

    # Derived quantities students can either use or verify themselves.
    out["ground_speed_m_s"] = np.sqrt(
        out["vn_m_s"] ** 2 + out["ve_m_s"] ** 2
    )
    out["course_deg"] = np.degrees(
        np.arctan2(out["ve_m_s"], out["vn_m_s"])
    )

    # Optional SITL truth attitude.
    truth_name, truth = choose_truth_attitude_source(dfs)
    if truth_name is not None:
        out["truth_roll_deg"] = interpolate_angle_deg(
            truth, target_t, FIELD_ALIASES["roll"]
        )
        out["truth_pitch_deg"] = interpolate_angle_deg(
            truth, target_t, FIELD_ALIASES["pitch"]
        )
        out["truth_yaw_deg"] = interpolate_angle_deg(
            truth, target_t, FIELD_ALIASES["yaw"]
        )

    return out


def build_all_student_dataset(
    motion: pd.DataFrame,
    complementary: pd.DataFrame,
) -> pd.DataFrame:
    """
    Create one convenience CSV on the native IMU timeline.

    The motion data are interpolated onto the complementary-filter/IMU
    timeline. This is useful when students prefer a single file.
    """
    out = complementary.copy()
    target_t = out["t_s"].to_numpy(dtype=float)

    if motion.empty:
        return out

    motion_t = motion["t_s"].to_numpy(dtype=float)

    for col in motion.columns:
        if col == "t_s" or col in out.columns:
            continue

        y = pd.to_numeric(motion[col], errors="coerce").to_numpy(dtype=float)
        valid = np.isfinite(motion_t) & np.isfinite(y)

        if valid.sum() < 2:
            out[col] = np.nan
            continue

        out[col] = np.interp(
            target_t,
            motion_t[valid],
            y[valid],
            left=np.nan,
            right=np.nan,
        )

    return out


# ---------------------------------------------------------------------------
# Output / validation
# ---------------------------------------------------------------------------

def save_raw_tables(
    dfs: Dict[str, pd.DataFrame],
    output_dir: Path,
    stem: str,
) -> None:
    for msg_type, df in dfs.items():
        if df.empty:
            continue
        df.to_csv(
            output_dir / f"{stem}_raw_{msg_type.lower()}.csv",
            index=False,
        )


def write_readme(
    path: Path,
    motion: pd.DataFrame,
    complementary: pd.DataFrame,
) -> None:
    truth_available = (
        "truth_source" in complementary.columns
        and complementary["truth_source"].iloc[0] != "NOT_AVAILABLE"
    )

    truth_text = (
        f"YES ({complementary['truth_source'].iloc[0]})"
        if truth_available
        else "NO - SIM/SIM2 attitude was not present in the log"
    )

    text = f"""STUDENT FLIGHT DATASET
======================

Generated files
---------------
1. {path.stem.replace('_README','')}_vehicle_motion.csv
   Intended for the Vehicle Motion and Sensor Investigation.

2. {path.stem.replace('_README','')}_complementary_filter.csv
   Intended for development of the gyro-only, accelerometer-only, and
   complementary attitude estimators.

3. {path.stem.replace('_README','')}_student_all.csv
   Convenience file containing the complementary-filter data plus
   interpolated motion variables.

Units / frames
--------------
t_s                 seconds from the common beginning of the extracted log
dt_s                seconds between adjacent IMU samples

north_m             NED North position [m]
east_m              NED East position [m]
down_m              NED Down position [m]

vn_m_s              NED North velocity [m/s]
ve_m_s              NED East velocity [m/s]
vd_m_s              NED Down velocity [m/s]

roll_deg             vehicle roll [deg]
pitch_deg            vehicle pitch [deg]
yaw_deg              vehicle yaw [deg]

gyro_x_rad_s         body x angular rate [rad/s]
gyro_y_rad_s         body y angular rate [rad/s]
gyro_z_rad_s         body z angular rate [rad/s]

accel_x_m_s2         body x accelerometer measurement [m/s^2]
accel_y_m_s2         body y accelerometer measurement [m/s^2]
accel_z_m_s2         body z accelerometer measurement [m/s^2]

ekf_roll_deg         XKF1 EKF roll estimate [deg]
ekf_pitch_deg        XKF1 EKF pitch estimate [deg]
ekf_yaw_deg          XKF1 EKF yaw estimate [deg]

truth_roll_deg       SITL truth roll [deg], when available
truth_pitch_deg      SITL truth pitch [deg], when available
truth_yaw_deg        SITL truth yaw [deg], when available

ground_speed_m_s     sqrt(VN^2 + VE^2)
course_deg            atan2(VE, VN), in degrees

SITL truth attitude available: {truth_text}

Recommended homework use
------------------------
Vehicle Motion:
    Use *_vehicle_motion.csv. Students can inspect NED position/velocity,
    attitude, body angular rates, and acceleration.

Complementary Filter:
    Use *_complementary_filter.csv. Students should derive their own
    accelerometer attitude, integrate gyro measurements, implement their own
    complementary filter, and compare against ekf_roll_deg/ekf_pitch_deg.
    If truth_* fields are populated, they can also compute error against
    simulation ground truth.

Important:
    The generated CSV intentionally does NOT contain a precomputed
    complementary-filter estimate. Students must implement that themselves.
"""
    path.write_text(text, encoding="utf-8")


def print_dataset_summary(
    name: str,
    df: pd.DataFrame,
) -> None:
    print(f"\n{name}")
    print("-" * len(name))
    print(f"Rows: {len(df)}")
    if "t_s" in df.columns and len(df):
        print(
            f"Time span: {df['t_s'].min():.3f} to "
            f"{df['t_s'].max():.3f} s"
        )

    populated = [
        c for c in df.columns
        if df[c].notna().any()
    ]
    missing = [
        c for c in df.columns
        if not df[c].notna().any()
    ]

    print("Populated columns:")
    print(", ".join(populated))

    if missing:
        print("Columns with no data:")
        print(", ".join(missing))


def make_csvs(
    log_path: str | Path,
    output_dir: Optional[str | Path] = None,
    motion_rate_hz: float = 20.0,
    save_raw: bool = False,
) -> Dict[str, Path]:
    log_path = Path(log_path)

    if output_dir is None:
        out_dir = log_path.parent
    else:
        out_dir = Path(output_dir)

    out_dir.mkdir(parents=True, exist_ok=True)
    stem = log_path.stem

    parser = FlightParser(log_path)
    dfs = parser.get_desired_data(DEFAULT_TYPES)

    motion = build_vehicle_motion_dataset(
        dfs,
        rate_hz=motion_rate_hz,
    )
    complementary = build_complementary_filter_dataset(dfs)
    all_data = build_all_student_dataset(motion, complementary)

    motion_path = out_dir / f"{stem}_vehicle_motion.csv"
    complementary_path = out_dir / f"{stem}_complementary_filter.csv"
    all_path = out_dir / f"{stem}_student_all.csv"
    readme_path = out_dir / f"{stem}_README.txt"

    motion.to_csv(motion_path, index=False)
    complementary.to_csv(complementary_path, index=False)
    all_data.to_csv(all_path, index=False)

    write_readme(readme_path, motion, complementary)

    if save_raw:
        save_raw_tables(dfs, out_dir, stem)

    print_dataset_summary("Vehicle Motion Dataset", motion)
    print_dataset_summary("Complementary Filter Dataset", complementary)

    print("\nSaved:")
    print(f"  {motion_path}")
    print(f"  {complementary_path}")
    print(f"  {all_path}")
    print(f"  {readme_path}")

    return {
        "vehicle_motion": motion_path,
        "complementary_filter": complementary_path,
        "all": all_path,
        "readme": readme_path,
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Parse an ArduPilot DataFlash BIN log into student-ready CSV "
            "files for the Vehicle Motion and Complementary Filter homework."
        )
    )
    parser.add_argument(
        "log",
        type=str,
        help="Path to the ArduPilot .BIN DataFlash log",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Directory for generated CSV files (default: log directory)",
    )
    parser.add_argument(
        "--motion-rate",
        type=float,
        default=30.0,
        help="Vehicle-motion CSV sample rate in Hz (default: 30)",
    )
    parser.add_argument(
        "--save-raw",
        action="store_true",
        help="Also save each extracted raw DataFlash message as a CSV",
    )

    args = parser.parse_args()

    make_csvs(
        log_path=args.log,
        output_dir=args.output_dir,
        motion_rate_hz=args.motion_rate,
        save_raw=args.save_raw,
    )


if __name__ == "__main__":
    main()
