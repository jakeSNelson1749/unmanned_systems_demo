#!/usr/bin/env bash

set -e

# ============================================================
# ArduPilot / Webots SITL launcher
# ============================================================

# Path to your ArduPilot repository.
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/coding_projects/ros2_trajectory_docker/ardupilot}"

# ArduPilot vehicle type
VEHICLE="${VEHICLE:-ArduCopter}"

# Webots simulator model
MODEL="${MODEL:-webots-python}"

# Address of the machine running Webots simulator
# If you are running on Linux change this to 127.0.0.1
# If not and running WSL2 change this the IP address of WSL by opening up
# Windows Terminal and entering ipconfig to find the WSL IP 
SIM_ADDRESS="${SIM_ADDRESS:-172.23.192.1}"

# Parameter file relative to the ArduPilot repository
PARAM_FILE="${PARAM_FILE:-libraries/SITL/examples/Webots_Python/params/iris.parm}"

# Set to false if you do not want to wipe parameters on startup
WIPE_PARAMS="${WIPE_PARAMS:-true}"

# ============================================================
# MAVLink outputs
# ============================================================

MAVLINK_OUTPUTS=(
    "127.0.0.1:14550"
    "127.0.0.1:14551"
    "127.0.0.1:14552"
    "127.0.0.1:14553"
)

# ============================================================
# Validate paths
# ============================================================

SIM_VEHICLE="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
PARAM_PATH="$ARDUPILOT_DIR/$PARAM_FILE"

if [[ ! -d "$ARDUPILOT_DIR" ]]; then
    echo "ERROR: ArduPilot directory does not exist:"
    echo "  $ARDUPILOT_DIR"
    exit 1
fi

if [[ ! -f "$SIM_VEHICLE" ]]; then
    echo "ERROR: sim_vehicle.py was not found:"
    echo "  $SIM_VEHICLE"
    exit 1
fi

if [[ ! -f "$PARAM_PATH" ]]; then
    echo "ERROR: parameter file was not found:"
    echo "  $PARAM_PATH"
    exit 1
fi

# ============================================================
# Build command
# ============================================================

CMD=(
    "./Tools/autotest/sim_vehicle.py"
    "--map"
    "--console"
    "-v" "$VEHICLE"
    "--model" "$MODEL"
    "--sim-address=$SIM_ADDRESS"
    "--add-param-file=$PARAM_PATH"
)

if [[ "$WIPE_PARAMS" == "true" ]]; then
    CMD+=("-w")
fi

for endpoint in "${MAVLINK_OUTPUTS[@]}"; do
    CMD+=("--out=$endpoint")
done

# ============================================================
# Print configuration
# ============================================================

echo "============================================================"
echo " ArduPilot Webots SITL"
echo "============================================================"
echo "ArduPilot:   $ARDUPILOT_DIR"
echo "Vehicle:     $VEHICLE"
echo "Model:       $MODEL"
echo "Webots IP:   $SIM_ADDRESS"
echo "Parameters:  $PARAM_PATH"
echo
echo "MAVLink outputs:"

for endpoint in "${MAVLINK_OUTPUTS[@]}"; do
    echo "  -> $endpoint"
done

echo
echo "Starting SITL..."
echo "============================================================"

# ============================================================
# Run from ArduPilot root
# ============================================================

cd "$ARDUPILOT_DIR"

exec "${CMD[@]}"