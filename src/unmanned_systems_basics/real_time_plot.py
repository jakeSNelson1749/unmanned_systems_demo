#!/usr/bin/env python3
# pip install pymavlink pyqtgraph PyQt5
import sys
import csv
import math
import time
from collections import deque
from typing import Dict

from pymavlink import mavutil
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


# ============================================================
# Configuration Change this to match the connection string from 
# our simulator
# ============================================================

MAVLINK_CONNECTION = "udp:127.0.0.1:14553"

PLOT_WINDOW_SEC = 50.0
UPDATE_RATE_HZ = 50.0

LOG_TO_CSV = True
CSV_FILE = "telemetry.csv"

# ============================================================
# MAVLink Telemetry Receiver
# ============================================================

class TelemetryReceiver:
    def __init__(self, connection_string):

        print(f"Connecting to {connection_string}...")

        self.master = mavutil.mavlink_connection(connection_string)

        self.master.wait_heartbeat()

        print(
            f"Connected to system {self.master.target_system}, "
            f"component {self.master.target_component}"
        )

        # Ask ArduPilot for telemetry streams
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            int(UPDATE_RATE_HZ),
            1
        )

        # Explicitly request IMU/raw sensor telemetry.
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            int(UPDATE_RATE_HZ),
            1
        )

        self.state:Dict[str, float] = {
            "n": 0.0,
            "e": 0.0,
            "d": 0.0,

            "vn": 0.0,
            "ve": 0.0,
            "vd": 0.0,

            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,

            "p": 0.0,
            "q": 0.0,
            "r": 0.0,

            # IMU accelerometer, normalized to m/s^2
            "accel_x": 0.0,
            "accel_y": 0.0,
            "accel_z": 0.0,

            # IMU gyroscope, normalized to deg/s
            "gyro_x": 0.0,
            "gyro_y": 0.0,
            "gyro_z": 0.0,
        }

    def update(self) -> Dict[str, float]:
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()

            # ------------------------------------------------
            # Local Position + Velocity
            # ------------------------------------------------

            if msg_type == "LOCAL_POSITION_NED":
                self.state["n"] = msg.x
                self.state["e"] = msg.y
                self.state["d"] = msg.z

                self.state["vn"] = msg.vx
                self.state["ve"] = msg.vy
                self.state["vd"] = msg.vz

            # ------------------------------------------------
            # Attitude + Angular Rates
            # ------------------------------------------------

            elif msg_type == "ATTITUDE":

                self.state["roll"] = math.degrees(msg.roll)
                self.state["pitch"] = math.degrees(msg.pitch)
                self.state["yaw"] = math.degrees(msg.yaw)

                self.state["p"] = math.degrees(msg.rollspeed)
                self.state["q"] = math.degrees(msg.pitchspeed)
                self.state["r"] = math.degrees(msg.yawspeed)

            # ------------------------------------------------
            # IMU Accelerometer + Gyroscope
            # ------------------------------------------------
            #
            # RAW_IMU / SCALED_IMU:
            #   accel = milli-g
            #   gyro  = milliradian/s
            #
            # HIGHRES_IMU:
            #   accel = m/s^2
            #   gyro  = rad/s
            #
            # Everything is normalized here to:
            #   accel -> m/s^2
            #   gyro  -> deg/s
            # ------------------------------------------------

            elif msg_type in ("RAW_IMU", "SCALED_IMU"):
                mg_to_mps2 = 9.80665 / 1000.0
                mrad_to_deg = 180.0 / (math.pi * 1000.0)

                self.state["accel_x"] = msg.xacc * mg_to_mps2
                self.state["accel_y"] = msg.yacc * mg_to_mps2
                self.state["accel_z"] = msg.zacc * mg_to_mps2

                self.state["gyro_x"] = msg.xgyro * mrad_to_deg
                self.state["gyro_y"] = msg.ygyro * mrad_to_deg
                self.state["gyro_z"] = msg.zgyro * mrad_to_deg

            elif msg_type == "HIGHRES_IMU":
                self.state["accel_x"] = msg.xacc
                self.state["accel_y"] = msg.yacc
                self.state["accel_z"] = msg.zacc

                self.state["gyro_x"] = math.degrees(msg.xgyro)
                self.state["gyro_y"] = math.degrees(msg.ygyro)
                self.state["gyro_z"] = math.degrees(msg.zgyro)

        return self.state

# ============================================================
# Main Dashboard
# ============================================================

class TelemetryDashboard(QtWidgets.QMainWindow):
    def __init__(self, receiver):

        super().__init__()

        self.receiver = receiver

        self.setWindowTitle("ArduPilot Live Telemetry Viewer")
        self.resize(1100, 900)

        self.start_time = time.time()

        # ====================================================
        # Data Storage
        # ====================================================

        max_samples = int(PLOT_WINDOW_SEC * UPDATE_RATE_HZ)

        self.time_data = deque(maxlen=max_samples)

        self.data = {
            "n": deque(maxlen=max_samples),
            "e": deque(maxlen=max_samples),
            "d": deque(maxlen=max_samples),

            "vn": deque(maxlen=max_samples),
            "ve": deque(maxlen=max_samples),
            "vd": deque(maxlen=max_samples),

            "roll": deque(maxlen=max_samples),
            "pitch": deque(maxlen=max_samples),
            "yaw": deque(maxlen=max_samples),

            "p": deque(maxlen=max_samples),
            "q": deque(maxlen=max_samples),
            "r": deque(maxlen=max_samples),

            "accel_x": deque(maxlen=max_samples),
            "accel_y": deque(maxlen=max_samples),
            "accel_z": deque(maxlen=max_samples),

            "gyro_x": deque(maxlen=max_samples),
            "gyro_y": deque(maxlen=max_samples),
            "gyro_z": deque(maxlen=max_samples),
        }

        # ====================================================
        # CSV Logging
        # ====================================================

        self.csv_file = None
        self.csv_writer = None

        if LOG_TO_CSV:

            self.csv_file = open(
                CSV_FILE,
                "w",
                newline=""
            )

            self.csv_writer = csv.writer(self.csv_file)

            self.csv_writer.writerow([
                "time_s",
                "north_m",
                "east_m",
                "down_m",
                "vn_mps",
                "ve_mps",
                "vd_mps",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "p_deg_s",
                "q_deg_s",
                "r_deg_s",
                "accel_x_mps2",
                "accel_y_mps2",
                "accel_z_mps2",
                "gyro_x_deg_s",
                "gyro_y_deg_s",
                "gyro_z_deg_s",
            ])

        # ====================================================
        # Central Widget
        # ====================================================

        central_widget = QtWidgets.QWidget()

        self.setCentralWidget(central_widget)

        main_layout = QtWidgets.QVBoxLayout(central_widget)

        # ====================================================
        # Top Controls
        # ====================================================
        control_layout = QtWidgets.QHBoxLayout()
        mode_label = QtWidgets.QLabel("Telemetry View:")
        self.mode_selector = QtWidgets.QComboBox()
        self.mode_selector.addItems([
            "North / South Motion",
            "East / West Motion",
            "Vertical Motion",
            "Heading / Yaw",
            "IMU Accelerometer",
            "IMU Gyroscope"
        ])

        self.mode_selector.currentIndexChanged.connect(
            self.configure_plots
        )

        control_layout.addWidget(mode_label)
        control_layout.addWidget(self.mode_selector)
        control_layout.addStretch()

        main_layout.addLayout(control_layout)

        # ====================================================
        # Current State Display
        # ====================================================

        self.state_label = QtWidgets.QLabel()

        self.state_label.setStyleSheet("""
            QLabel {
                font-family: monospace;
                font-size: 16px;
                padding: 8px;
            }
        """)

        main_layout.addWidget(self.state_label)

        # ====================================================
        # Dedicated IMU / Gyro Live Readout
        # ====================================================

        imu_group = QtWidgets.QGroupBox("IMU Sensor Readings")
        imu_layout = QtWidgets.QGridLayout(imu_group)

        self.imu_value_labels = {}

        imu_fields = [
            ("accel_x", "Accel X", "m/s²"),
            ("accel_y", "Accel Y", "m/s²"),
            ("accel_z", "Accel Z", "m/s²"),
            ("gyro_x", "Gyro X", "deg/s"),
            ("gyro_y", "Gyro Y", "deg/s"),
            ("gyro_z", "Gyro Z", "deg/s"),
        ]

        for index, (key, title, units) in enumerate(imu_fields):
            row = index // 3
            col = index % 3

            field_widget = QtWidgets.QWidget()
            field_layout = QtWidgets.QVBoxLayout(field_widget)
            field_layout.setContentsMargins(6, 4, 6, 4)

            title_label = QtWidgets.QLabel(title)
            title_label.setAlignment(QtCore.Qt.AlignCenter)
            title_label.setStyleSheet(
                "font-size: 13px; font-weight: bold;"
            )

            value_label = QtWidgets.QLabel(f"0.000 {units}")
            value_label.setAlignment(QtCore.Qt.AlignCenter)
            value_label.setStyleSheet("""
                QLabel {
                    font-family: monospace;
                    font-size: 18px;
                    padding: 6px;
                    border: 1px solid #777;
                    border-radius: 4px;
                }
            """)

            field_layout.addWidget(title_label)
            field_layout.addWidget(value_label)

            imu_layout.addWidget(field_widget, row, col)
            self.imu_value_labels[key] = (value_label, units)

        main_layout.addWidget(imu_group)

        # ====================================================
        # Plot Widget
        # ====================================================

        self.graphics_layout = pg.GraphicsLayoutWidget()

        main_layout.addWidget(self.graphics_layout)

        # ====================================================
        # Create 3 Plots
        # ====================================================

        self.plot1 = self.graphics_layout.addPlot(row=0, col=0)
        self.plot2 = self.graphics_layout.addPlot(row=1, col=0)
        self.plot3 = self.graphics_layout.addPlot(row=2, col=0)

        self.plot1.showGrid(x=True, y=True)
        self.plot2.showGrid(x=True, y=True)
        self.plot3.showGrid(x=True, y=True)

        # Keep axis units fixed exactly as specified in setLabel().
        #
        # PyQtGraph normally applies automatic SI prefixes based on the
        # displayed range, which can change:
        #
        #     m/s   -> mm/s
        #     deg/s -> mdeg/s
        #     s     -> ms
        #
        # Disable that behavior so the dashboard always displays the
        # original engineering units.
        for plot in (self.plot1, self.plot2, self.plot3):
            plot.getAxis("left").enableAutoSIPrefix(False)
            plot.getAxis("bottom").enableAutoSIPrefix(False)

        self.curve1 = self.plot1.plot()
        self.curve2 = self.plot2.plot()
        self.curve3 = self.plot3.plot()

        # ====================================================
        # Initial Plot Configuration
        # ====================================================

        self.configure_plots()

        # ====================================================
        # Timer
        # ====================================================

        self.timer = QtCore.QTimer()

        self.timer.timeout.connect(
            self.update_dashboard
        )

        self.timer.start(
            int(1000 / UPDATE_RATE_HZ)
        )

    # ========================================================
    # Plot Configuration
    # ========================================================

    def configure_plots(self):

        mode = self.mode_selector.currentIndex()

        self.plot1.clear()
        self.plot2.clear()
        self.plot3.clear()

        self.curve1 = self.plot1.plot()
        self.curve2 = self.plot2.plot()
        self.curve3 = self.plot3.plot()

        # ----------------------------------------------------
        # North / South
        # ----------------------------------------------------

        if mode == 0:

            self.plot1.setTitle(
                "North Position"
            )

            self.plot1.setLabel(
                "left",
                "North",
                units="m"
            )

            self.plot2.setTitle(
                "North Velocity"
            )

            self.plot2.setLabel(
                "left",
                "V_N",
                units="m/s"
            )

            self.plot3.setTitle(
                "Pitch"
            )

            self.plot3.setLabel(
                "left",
                "Pitch",
                units="deg"
            )

        # ----------------------------------------------------
        # East / West
        # ----------------------------------------------------

        elif mode == 1:

            self.plot1.setTitle(
                "East Position"
            )

            self.plot1.setLabel(
                "left",
                "East",
                units="m"
            )

            self.plot2.setTitle(
                "East Velocity"
            )

            self.plot2.setLabel(
                "left",
                "V_E",
                units="m/s"
            )

            self.plot3.setTitle(
                "Roll"
            )

            self.plot3.setLabel(
                "left",
                "Roll",
                units="deg"
            )

        # ----------------------------------------------------
        # Vertical Motion
        # ----------------------------------------------------

        elif mode == 2:

            self.plot1.setTitle(
                "Down Position"
            )

            self.plot1.setLabel(
                "left",
                "Down",
                units="m"
            )

            self.plot2.setTitle(
                "Vertical Velocity"
            )

            self.plot2.setLabel(
                "left",
                "V_D",
                units="m/s"
            )

            self.plot3.setTitle(
                "Pitch"
            )

            self.plot3.setLabel(
                "left",
                "Pitch",
                units="deg"
            )

        # ----------------------------------------------------
        # Heading
        # ----------------------------------------------------

        elif mode == 3:

            self.plot1.setTitle(
                "Yaw / Heading"
            )

            self.plot1.setLabel(
                "left",
                "Yaw",
                units="deg"
            )

            self.plot2.setTitle(
                "Yaw Rate"
            )

            self.plot2.setLabel(
                "left",
                "r",
                units="deg/s"
            )

            self.plot3.setTitle(
                "North Velocity"
            )

            self.plot3.setLabel(
                "left",
                "V_N",
                units="m/s"
            )

        # ----------------------------------------------------
        # IMU Accelerometer
        # ----------------------------------------------------

        elif mode == 4:

            self.plot1.setTitle("Accelerometer X")
            self.plot1.setLabel("left", "Accel X", units="m/s^2")

            self.plot2.setTitle("Accelerometer Y")
            self.plot2.setLabel("left", "Accel Y", units="m/s^2")

            self.plot3.setTitle("Accelerometer Z")
            self.plot3.setLabel("left", "Accel Z", units="m/s^2")

        # ----------------------------------------------------
        # IMU Gyroscope
        # ----------------------------------------------------

        elif mode == 5:

            self.plot1.setTitle("Gyroscope X")
            self.plot1.setLabel("left", "Gyro X", units="deg/s")

            self.plot2.setTitle("Gyroscope Y")
            self.plot2.setLabel("left", "Gyro Y", units="deg/s")

            self.plot3.setTitle("Gyroscope Z")
            self.plot3.setLabel("left", "Gyro Z", units="deg/s")

        self.plot1.setLabel(
            "bottom",
            "Time",
            units="s"
        )

        self.plot2.setLabel(
            "bottom",
            "Time",
            units="s"
        )

        self.plot3.setLabel(
            "bottom",
            "Time",
            units="s"
        )

    # ========================================================
    # Dashboard Update
    # ========================================================

    def update_dashboard(self):

        state = self.receiver.update()

        elapsed = time.time() - self.start_time

        self.time_data.append(elapsed)

        for key in self.data:
            self.data[key].append(
                state[key]
            )

        # ----------------------------------------------------
        # Update Current State Text
        # ----------------------------------------------------

        state_text = (
            f"POSITION NED   "
            f"N: {state['n']:7.2f} m   "
            f"E: {state['e']:7.2f} m   "
            f"D: {state['d']:7.2f} m\n"

            f"VELOCITY NED   "
            f"VN: {state['vn']:7.2f} m/s   "
            f"VE: {state['ve']:7.2f} m/s   "
            f"VD: {state['vd']:7.2f} m/s\n"

            f"ATTITUDE       "
            f"Roll: {state['roll']:7.2f} deg   "
            f"Pitch: {state['pitch']:7.2f} deg   "
            f"Yaw: {state['yaw']:7.2f} deg"
        )

        self.state_label.setText(
            state_text
        )

        # ----------------------------------------------------
        # Update Dedicated IMU / Gyro Readout
        # ----------------------------------------------------

        for key in ("accel_x", "accel_y", "accel_z"):
            label, units = self.imu_value_labels[key]
            label.setText(f"{state[key]:8.3f} {units}")

        for key in ("gyro_x", "gyro_y", "gyro_z"):
            label, units = self.imu_value_labels[key]
            label.setText(f"{state[key]:8.2f} {units}")

        # ----------------------------------------------------
        # Plot Selected Signals
        # ----------------------------------------------------

        mode = self.mode_selector.currentIndex()

        t = list(self.time_data)

        if mode == 0:

            self.curve1.setData(
                t,
                list(self.data["n"])
            )

            self.curve2.setData(
                t,
                list(self.data["vn"])
            )

            self.curve3.setData(
                t,
                list(self.data["pitch"])
            )

        elif mode == 1:

            self.curve1.setData(
                t,
                list(self.data["e"])
            )

            self.curve2.setData(
                t,
                list(self.data["ve"])
            )

            self.curve3.setData(
                t,
                list(self.data["roll"])
            )

        elif mode == 2:

            self.curve1.setData(
                t,
                list(self.data["d"])
            )

            self.curve2.setData(
                t,
                list(self.data["vd"])
            )

            self.curve3.setData(
                t,
                list(self.data["pitch"])
            )

        elif mode == 3:

            self.curve1.setData(
                t,
                list(self.data["yaw"])
            )

            self.curve2.setData(
                t,
                list(self.data["r"])
            )

            self.curve3.setData(
                t,
                list(self.data["vn"])
            )

        elif mode == 4:

            self.curve1.setData(
                t,
                list(self.data["accel_x"])
            )

            self.curve2.setData(
                t,
                list(self.data["accel_y"])
            )

            self.curve3.setData(
                t,
                list(self.data["accel_z"])
            )

        elif mode == 5:

            self.curve1.setData(
                t,
                list(self.data["gyro_x"])
            )

            self.curve2.setData(
                t,
                list(self.data["gyro_y"])
            )

            self.curve3.setData(
                t,
                list(self.data["gyro_z"])
            )

        # ----------------------------------------------------
        # CSV Logging
        # ----------------------------------------------------

        if self.csv_writer is not None:

            self.csv_writer.writerow([
                elapsed,

                state["n"],
                state["e"],
                state["d"],

                state["vn"],
                state["ve"],
                state["vd"],

                state["roll"],
                state["pitch"],
                state["yaw"],

                state["p"],
                state["q"],
                state["r"],

                state["accel_x"],
                state["accel_y"],
                state["accel_z"],

                state["gyro_x"],
                state["gyro_y"],
                state["gyro_z"],
            ])

            self.csv_file.flush()

    # ========================================================
    # Clean Shutdown
    # ========================================================

    def closeEvent(self, event):

        if self.csv_file is not None:

            self.csv_file.close()

            print(
                f"Telemetry saved to {CSV_FILE}"
            )

        event.accept()