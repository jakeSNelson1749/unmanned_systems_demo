from unmanned_systems_basics.quadcopter import Quadcopter

vehicle = Quadcopter("udp:127.0.0.1:14551")

while True:
    state = vehicle.update()

    print(
        f"NED position: "
        f"{state.north_m:.2f}, "
        f"{state.east_m:.2f}, "
        f"{state.down_m:.2f}"
    )

    print(
        f"NED velocity: "
        f"{state.vn_m_s:.2f}, "
        f"{state.ve_m_s:.2f}, "
        f"{state.vd_m_s:.2f}"
    )

    print(
        f"Attitude: "
        f"{state.roll_deg:.1f}, "
        f"{state.pitch_deg:.1f}, "
        f"{state.yaw_deg:.1f}"
    )