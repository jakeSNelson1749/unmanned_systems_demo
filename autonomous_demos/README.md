# Quadcopter SITL Class Examples

These examples are intentionally small to understand the basic concepts of listening to sensors and controlling the quadcopter/drone.

## Files

```text
quadcopter.py
autonomous_demos/
    read_state.py
    command_velocity.py
    command_attitude.py
```

`src/unmanned_systems_basics/quadcopter.py` contains the instructor-provided `Quadcopter` class.

The example scripts show how to use that class.

ArduCopter SITL should already be running and exposing MAVLink at the
connection used by the scripts.

The default connection is:

```text
udp:127.0.0.1:14550
```

Adjust the connection string if your SITL configuration uses a different
endpoint.

## 1. Read Vehicle State

Run:

```bash
uv run autonomous_demos/read_state.py
```

This displays:

- local NED position
- local NED velocity
- roll
- pitch
- yaw

The convention is:

```text
+North
+East
+Down
```

Therefore a negative Down velocity corresponds to climbing.

## 2. Command NED Velocity

Run:

```bash
uv run examples/command_velocity.py
```

The script demonstrates:

```python
vehicle.command_velocity_ned(
    vn_m_s=2.0,
    ve_m_s=0.0,
    vd_m_s=0.0,
)
```

This is the command layer used by the required undergraduate position-control
problem.

Students will later replace the hard-coded velocity command with:

```text
desired position
        |
        v
position error
        |
        v
student position controller
        |
        v
VN_cmd, VE_cmd, VD_cmd
        |
        v
command_velocity_ned(...)
```

ArduPilot retains the lower-level velocity, attitude, thrust, and motor
control.

## 3. Command Attitude

Run:

```bash
uv run examples/command_attitude.py
```

The script demonstrates:

```python
vehicle.command_attitude(
    roll_deg=10.0,
    pitch_deg=0.0,
    yaw_deg=current_yaw,
    thrust=0.5,
)
```

This is useful for demonstrating how roll and pitch affect horizontal
vehicle motion.

It also provides the lower command layer needed by the graduate extension:

```text
desired velocity
       |
       v
velocity error
       |
       v
student velocity controller
       |
       v
roll / pitch command
       |
       v
command_attitude(...)
```

The example does **not** provide the velocity-to-attitude mapping. Graduate
students are expected to develop that relationship themselves while accounting
for the course NED/body-frame conventions and the effect of yaw.
