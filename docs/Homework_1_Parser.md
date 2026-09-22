# Parsing ArduPilot Flight Data for Homework 1

Homework 1 uses ArduPilot DataFlash `.BIN` logs for the **Vehicle Motion and Sensor Investigation** and **Complementary Filter** exercises.

The provided parser:

```text
parse_student_flight_data.py
```

extracts the required flight data and converts it into student-ready CSV files.

SITL `.BIN` files are typically saved where ardupilot is at so ie if you want to get the quadcopter files they would be in `ardupilot/ArduCopter/logs`

## Running the Parser

From the directory containing the parser, run:

```bash
uv run parse_student_flight_data.py path/to/flight.BIN
```

For example:

```bash
uv run parse_student_flight_data.py data/00000005.BIN
```

The generated files will be placed in the same directory as the `.BIN` file unless a different output directory is specified.

## Generated Files

Running the parser produces files similar to:

```text
00000005_vehicle_motion.csv
00000005_complementary_filter.csv
00000005_student_all.csv
00000005_README.md
```

### Vehicle Motion Dataset

Use:

```text
00000005_vehicle_motion.csv
```

for the **Vehicle Motion and Sensor Investigation** portion of Homework 1.

This dataset contains:

- NED position
- NED velocity
- roll, pitch, and yaw
- body angular rates
- accelerometer measurements
- course angle
- horizontal ground speed
- ArduPilot EKF attitude
- SITL ground-truth attitude when available

### Complementary Filter Dataset

Use:

```text
00000005_complementary_filter.csv
```

for the **Complementary Filter Development** and **Complementary Filter vs. ArduPilot EKF** portions of Homework 1.

This dataset contains:

- IMU gyroscope measurements
- IMU accelerometer measurements
- sample time (`dt_s`)
- ArduPilot EKF roll, pitch, and yaw
- autopilot attitude output
- SITL ground-truth attitude when available

The complementary-filter estimate is intentionally **not** included. You are expected to implement this estimator themselves.

## Optional Arguments

Specify a different output directory:

```bash
python parse_student_flight_data.py data/00000005.BIN \
    --output-dir student_data
```

Change the sampling rate of the vehicle-motion dataset:

```bash
python parse_student_flight_data.py data/00000005.BIN \
    --motion-rate 20
```

Export the original DataFlash message tables for debugging:

```bash
python parse_student_flight_data.py data/00000005.BIN \
    --save-raw
```

Multiple options may be combined:

```bash
python parse_student_flight_data.py data/00000005.BIN \
    --output-dir student_data \
    --motion-rate 20 \
    --save-raw
```

## Coordinate and Unit Conventions

Unless otherwise specified:

| Quantity | Convention |
|---|---|
| Position | North-East-Down (NED) |
| Velocity | North-East-Down (NED) |
| Attitude | Degrees |
| Gyroscope | rad/s |
| Accelerometer | m/s² |
| Time | Seconds |

Students should pay particular attention to coordinate-frame and sign conventions when implementing the accelerometer-based attitude calculation.

## Important Notes/Tips
- The parser also generates a dataset-specific `README.md` containing a full description of the columns available in that particular flight log.

## Plotting Tips
Once the csv files are generated check out the `pandas` library to import your csv as a dataframe and leverage `matplotlib` to begin plotting your data

## Expected ArduPilot Log Messages

For the full Homework 1 dataset, the DataFlash log should ideally contain:

```text
IMU
ATT
XKF1
SIM or SIM2
GPS
BARO
MAG
```

`IMU`, `ATT`, and `XKF1` are the most important messages for the required homework analysis.

`SIM` or `SIM2` is required if true simulated attitude is to be included for ground-truth comparison.

## Troubleshooting

If a generated column contains only blank or `NaN` values, first inspect the parser output to confirm that the corresponding DataFlash message was present in the `.BIN` file.

You may also run:

```bash
python parse_student_flight_data.py path/to/flight.BIN --save-raw
```

and inspect the raw message CSV files.

Do not substitute another signal for a missing measurement without documenting and justifying the change.
