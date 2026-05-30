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

echo "=========================================================="
echo " Executing Absolute Trajectory Error (ATE / APE) Analysis"
echo "=========================================================="

evo_ape tum "$GROUNDTRUTH" "$ESTIMATED" -a --plot --plot_mode=xyz

echo ""
echo "=========================================================="
echo " Executing Relative Pose Error (RPE) Analysis"
echo "=========================================================="

evo_rpe tum "$GROUNDTRUTH" "$ESTIMATED" -a --plot --plot_mode=xyz
