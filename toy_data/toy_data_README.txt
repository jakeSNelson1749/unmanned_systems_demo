STUDENT FLIGHT DATASET
======================

Generated files
---------------
1. toy_data_vehicle_motion.csv
   Intended for the Vehicle Motion and Sensor Investigation.

2. toy_data_complementary_filter.csv
   Intended for development of the gyro-only, accelerometer-only, and
   complementary attitude estimators.

3. toy_data_student_all.csv
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

SITL truth attitude available: YES (SIM)

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
