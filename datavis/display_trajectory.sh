#!/bin/bash
# Display Trajectory Script
# This script preprocesses and visualizes robotics sensor data from TR00/TR02 files.
#
# Usage: ./display_trajectory.sh TR00
#        ./display_trajectory.sh TR02

# Check if trajectory name is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <trajectory_name>"
    echo "Example: $0 TR00"
    echo "         $0 TR02"
    exit 1
fi

TRAJECTORY=$1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# PYTHON_ENV="$SCRIPT_DIR/.venv/bin/python"
# PYTHON_ENV="$SCRIPT_DIR/.venv/bin/python"
PYTHON_ENV="/home/vincent/anaconda3/envs/imrclab/bin/python"


# Check if trajectory file exists
if [ ! -f "$SCRIPT_DIR/$TRAJECTORY" ]; then
    echo "Error: Trajectory file '$TRAJECTORY' not found in $SCRIPT_DIR"
    exit 1
fi

echo "=================================================="
echo "PROCESSING TRAJECTORY: $TRAJECTORY"
echo "=================================================="

# Step 1: Clean the CSV data
echo "Step 1: Preprocessing $TRAJECTORY data..."
$PYTHON_ENV -c "
import sys
sys.path.append('$SCRIPT_DIR')
from clean_trajectory import clean_single_trajectory
clean_single_trajectory('$TRAJECTORY')
"

# Check if preprocessing was successful
if [ $? -ne 0 ]; then
    echo "Error: Failed to preprocess $TRAJECTORY"
    exit 1
fi

# Step 2: Display the data
echo ""
echo "Step 2: Visualizing ${TRAJECTORY}_final.csv..."
$PYTHON_ENV display_data.py --${TRAJECTORY}_final.csv

echo ""
echo "=================================================="
echo "TRAJECTORY PROCESSING COMPLETE"
echo "=================================================="
