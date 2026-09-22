#!/usr/bin/env python3
"""
quadcopter.py

This module intentionally hides most pymavlink boilerplate so students can
focus on how different command layers affect quadcopter motion.

Provided interfaces:
    - read local NED position / velocity
    - read roll / pitch / yaw
    - command NED velocity
    - command local NED position (ArduPilot baseline)
    - command roll / pitch / yaw attitude
    - arm / disarm / set GUIDED mode
    - optional takeoff helper for establishing the initial condition

"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil


@dataclass
class VehicleState:
    """Latest vehicle state."""

    time_s: float = 0.0

    # Local NED position [m]
    north_m: float = 0.0
    east_m: float = 0.0
    down_m: float = 0.0

    # Local NED velocity [m/s]
    vn_m_s: float = 0.0
    ve_m_s: float = 0.0
    vd_m_s: float = 0.0

    # Euler attitude [deg]
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def euler_to_quaternion(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> list[float]:
    """Convert roll/pitch/yaw [deg] to quaternion [w, x, y, z]."""

    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)

    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


class Quadcopter:
    """
    Minimal ArduPilot SITL interface for class demonstrations and Homework 1.

    Typical usage:

        vehicle = Quadcopter("udp:127.0.0.1:14550")
        vehicle.set_mode("GUIDED")

        while True:
            state = vehicle.update()
            vehicle.command_velocity_ned(2.0, 0.0, 0.0)
    """

    def __init__(
        self,
        connection: str = "udp:127.0.0.1:14550",
        source_system: int = 255,
        max_horizontal_velocity_m_s: float = 5.0,
        max_vertical_velocity_m_s: float = 2.0,
        max_roll_deg: float = 20.0,
        max_pitch_deg: float = 20.0,
    ) -> None:

        self.connection_string = connection

        self.max_horizontal_velocity_m_s = max_horizontal_velocity_m_s
        self.max_vertical_velocity_m_s = max_vertical_velocity_m_s
        self.max_roll_deg = max_roll_deg
        self.max_pitch_deg = max_pitch_deg

        print(f"[Quadcopter] Connecting to {connection}")

        self.master = mavutil.mavlink_connection(
            connection,
            source_system=source_system,
        )

        print("[Quadcopter] Waiting for heartbeat...")
        self.master.wait_heartbeat()

        print(
            "[Quadcopter] Connected to "
            f"system={self.master.target_system}, "
            f"component={self.master.target_component}"
        )

        self.state = VehicleState()
        self._start_time = time.monotonic()

        # Heading used by command_velocity_ned() when yaw_deg is not
        # explicitly supplied.
        self._held_yaw_deg: Optional[float] = None

        self._request_telemetry()

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _request_telemetry(self) -> None:
        """
        Request the two messages
        LOCAL_POSITION_NED https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED
        ATTITUDE https://mavlink.io/en/messages/common.html#ATTITUDE 
        Specifies the frequencies at which the messages should be sent.
        """

        requested = {
            32: 30.0,  # LOCAL_POSITION_NED
            30: 30.0,  # ATTITUDE
        }

        for message_id, rate_hz in requested.items():
            interval_us = int(1e6 / rate_hz)

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                interval_us,
                0, 0, 0, 0, 0,
            )

    def set_mode(self, mode: str = "GUIDED") -> None:
        """Set the ArduCopter flight mode."""

        mode = mode.upper()
        mapping = self.master.mode_mapping()

        if mode not in mapping:
            raise ValueError(
                f"Mode '{mode}' is not available. "
                f"Available modes: {sorted(mapping.keys())}"
            )

        self.master.set_mode(mapping[mode])

        timeout = time.monotonic() + 5.0

        while time.monotonic() < timeout:
            msg = self.master.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=0.5,
            )

            if msg is None:
                continue

            if msg.custom_mode == mapping[mode]:
                print(f"[Quadcopter] Mode set to {mode}")
                return

        raise TimeoutError(f"Timed out waiting for mode {mode}")

    def arm(self) -> None:
        """Arm the simulated vehicle."""

        print("[Quadcopter] Arming...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("[Quadcopter] Armed")

    def disarm(self) -> None:
        """Disarm the simulated vehicle."""

        print("[Quadcopter] Disarming...")
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        print("[Quadcopter] Disarmed")

    def takeoff(self, altitude_m: float = 5.0) -> None:
        """
        Convenience helper to establish the initial test condition.

        Takeoff itself is not part of the control-learning objective.
        """

        self.set_mode("GUIDED")

        if not self.master.motors_armed():
            self.arm()

        print(f"[Quadcopter] Takeoff command: {altitude_m:.1f} m")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0,
            altitude_m,
        )

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def update(self, timeout_s: float = 0.02) -> VehicleState:
        """
        Consume available telemetry and return the latest state.

        Position and velocity use local NED:
            +North
            +East
            +Down
        """

        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            msg = self.master.recv_match(blocking=False)

            if msg is None:
                time.sleep(0.001)
                continue

            msg_type = msg.get_type()

            if msg_type == "LOCAL_POSITION_NED":
                self.state.north_m = float(msg.x)
                self.state.east_m = float(msg.y)
                self.state.down_m = float(msg.z)

                self.state.vn_m_s = float(msg.vx)
                self.state.ve_m_s = float(msg.vy)
                self.state.vd_m_s = float(msg.vz)

            elif msg_type == "ATTITUDE":
                self.state.roll_deg = math.degrees(msg.roll)
                self.state.pitch_deg = math.degrees(msg.pitch)
                self.state.yaw_deg = wrap_angle_deg(
                    math.degrees(msg.yaw)
                )

        self.state.time_s = time.monotonic() - self._start_time

        return self.state

    # ------------------------------------------------------------------
    # Heading hold
    # ------------------------------------------------------------------

    def hold_current_heading(self) -> float:
        """
        Capture the latest measured yaw and use it as the heading hold target.

        Call update() before this method if you want to guarantee that the
        stored attitude is fresh.
        """
        self._held_yaw_deg = wrap_angle_deg(self.state.yaw_deg)

        print(
            "[Quadcopter] Holding heading at "
            f"{self._held_yaw_deg:.1f} deg"
        )

        return self._held_yaw_deg

    def reset_heading_hold(self) -> None:
        """
        Clear the stored heading.

        The next call to command_velocity_ned(..., yaw_deg=None) will capture
        the vehicle's then-current yaw and hold that new heading.
        """
        self._held_yaw_deg = None

    # ------------------------------------------------------------------
    # Velocity command
    # ------------------------------------------------------------------

    def command_velocity_ned(
        self,
        vn_m_s: float,
        ve_m_s: float,
        vd_m_s: float,
        yaw_deg: Optional[float] = None,
    ) -> None:
        """
        Command local NED velocity.

        vn_m_s:
            North velocity [m/s]

        ve_m_s:
            East velocity [m/s]

        vd_m_s:
            Down velocity [m/s]
            +VD = downward
            -VD = upward

        yaw_deg:
            Optional desired absolute heading in degrees.

            If None, the first velocity command captures the vehicle's
            current yaw. That same heading is then commanded on every
            subsequent velocity command so the quadcopter can translate
            North/East/Down without turning to face the velocity vector.

            Call reset_heading_hold() to make the next velocity command
            capture a new current heading.
        """

        horizontal_speed = math.hypot(vn_m_s, ve_m_s)

        if horizontal_speed > self.max_horizontal_velocity_m_s:
            scale = (
                self.max_horizontal_velocity_m_s
                / horizontal_speed
            )

            vn_m_s *= scale
            ve_m_s *= scale

        vd_m_s = clamp(
            vd_m_s,
            -self.max_vertical_velocity_m_s,
            self.max_vertical_velocity_m_s,
        )

        # --------------------------------------------------------
        # Heading hold
        # --------------------------------------------------------
        #
        # MAV_FRAME_LOCAL_NED keeps VN/VE/VD in the world/NED frame.
        # Yaw is commanded separately so the vehicle can translate
        # without turning to face the direction of travel.
        #
        # Capture yaw only once. Re-reading current yaw every control
        # iteration would slowly accept yaw drift as the new target.
        # --------------------------------------------------------

        if yaw_deg is None:
            if self._held_yaw_deg is None:
                self._held_yaw_deg = wrap_angle_deg(
                    self.state.yaw_deg
                )

            commanded_yaw_deg = self._held_yaw_deg

        else:
            commanded_yaw_deg = wrap_angle_deg(yaw_deg)

            # An explicit yaw command becomes the new held heading.
            self._held_yaw_deg = commanded_yaw_deg

        yaw_rad = math.radians(commanded_yaw_deg)

        # --------------------------------------------------------
        # We are commanding:
        #
        #     velocity = USED
        #     position = IGNORED
        #     acceleration = IGNORED
        #     yaw = USED
        #     yaw rate = IGNORED
        # --------------------------------------------------------

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self.master.mav.set_position_target_local_ned_send(
            int((time.monotonic() - self._start_time) * 1000),

            self.master.target_system,
            self.master.target_component,

            mavutil.mavlink.MAV_FRAME_LOCAL_NED,

            type_mask,

            # Position ignored
            0.0,
            0.0,
            0.0,

            # Velocity used
            vn_m_s,
            ve_m_s,
            vd_m_s,

            # Acceleration ignored
            0.0,
            0.0,
            0.0,

            # Yaw used
            yaw_rad,

            # Yaw rate ignored
            0.0,
        )
    # ------------------------------------------------------------------
    # Position command
    # ------------------------------------------------------------------

    def command_position_ned(
        self,
        north_m: float,
        east_m: float,
        down_m: float,
        yaw_deg: Optional[float] = None,
    ) -> None:
        """
        Command local NED position using ArduPilot's built-in controllers.

        This is useful for demonstrating the difference between:
            1. commanding a position directly through ArduPilot, and
            2. the student homework controller that converts position
               error into velocity command.

        Students should not use this as the required Problem 4 solution.
        """

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        yaw_rad = 0.0

        if yaw_deg is None:
            type_mask |= (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            )
        else:
            yaw_rad = math.radians(yaw_deg)

        self.master.mav.set_position_target_local_ned_send(
            int((time.monotonic() - self._start_time) * 1000),
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            north_m,
            east_m,
            down_m,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            yaw_rad,
            0.0,
        )

    # ------------------------------------------------------------------
    # Attitude command
    # ------------------------------------------------------------------

    def command_attitude(
        self,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        thrust: float = 0.5,
    ) -> None:
        """
        Command roll, pitch, yaw, and normalized thrust.

        This interface is useful for the graduate extension where students
        map horizontal velocity error into attitude commands.

        ArduPilot still owns the internal attitude/rate control and motor
        mixing below this command layer.
        """

        roll_deg = clamp(
            roll_deg,
            -self.max_roll_deg,
            self.max_roll_deg,
        )

        pitch_deg = clamp(
            pitch_deg,
            -self.max_pitch_deg,
            self.max_pitch_deg,
        )

        thrust = clamp(thrust, 0.0, 1.0)

        q = euler_to_quaternion(
            roll_deg,
            pitch_deg,
            yaw_deg,
        )

        # Use attitude quaternion; ignore body-rate fields.
        type_mask = (
            getattr(
                mavutil.mavlink,
                "ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE",
                1,
            )
            | getattr(
                mavutil.mavlink,
                "ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE",
                2,
            )
            | getattr(
                mavutil.mavlink,
                "ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE",
                4,
            )
        )

        self.master.mav.set_attitude_target_send(
            int((time.monotonic() - self._start_time) * 1000),
            self.master.target_system,
            self.master.target_component,
            type_mask,
            q,
            0.0,
            0.0,
            0.0,
            thrust,
        )

    def stop(self) -> None:
        """Command zero NED velocity while continuing to hold heading."""
        self.command_velocity_ned(0.0, 0.0, 0.0)
