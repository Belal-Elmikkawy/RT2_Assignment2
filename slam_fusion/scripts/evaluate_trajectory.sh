#!/bin/bash
# evaluate_trajectory.sh

if [ "$#" -ne 2 ]; then
    echo "Usage: ./evaluate_trajectory.sh <estimated_trajectory_file> <groundtruth_file>"
    echo "Example: ./evaluate_trajectory.sh rtabmap_traj.txt /workspace/datasets/TUM/rgbd_dataset_freiburg1_desk/groundtruth.txt"
    exit 1
fi

ESTIMATED=$1
GROUNDTRUTH=$2

if [ ! -f "$ESTIMATED" ]; then
    echo "Error: Estimated trajectory file not found: $ESTIMATED"
    exit 1
fi

if [ ! -f "$GROUNDTRUTH" ]; then
    echo "Error: Groundtruth file not found: $GROUNDTRUTH"
    exit 1
fi

if [ ! -s "$ESTIMATED" ]; then
    echo "=========================================================="
    echo " [CRITICAL ERROR] The file $ESTIMATED is EMPTY (0 bytes)!"
    echo " This means ORB-SLAM3 did not save the trajectory properly,"
    echo " or it contained no valid data. Make sure you let the robot"
    echo " move around and then gracefully close the simulation (Ctrl+C)."
    echo "=========================================================="
    exit 1
fi

# Automatically clean the TUM trajectory file to fix any evo parser errors 
# caused by ORB-SLAM3 or trailing spaces/empty lines from abrupt shutdowns.
CLEANER_SCRIPT="/ros2_ws/src/slam_fusion/scripts/clean_tum_traj.py"
if [ -f "$CLEANER_SCRIPT" ]; then
    echo "Running trajectory cleaner..."
    python3 "$CLEANER_SCRIPT" "$ESTIMATED"
fi

if [ ! -s "$ESTIMATED" ]; then
    echo "=========================================================="
    echo " [CRITICAL ERROR] The file $ESTIMATED is EMPTY after cleaning!"
    echo " This means all lines in the file were malformed or not TUM format."
    echo " Please check the raw contents of $ESTIMATED."
    echo "=========================================================="
    exit 1
fi

evo_ape tum "$GROUNDTRUTH" "$ESTIMATED" -a --plot --plot_mode=xyz

echo ""
echo "=========================================================="
echo " Executing Relative Pose Error (RPE) Analysis"
echo "=========================================================="

evo_rpe tum "$GROUNDTRUTH" "$ESTIMATED" -a --plot --plot_mode=xyz
