#!/bin/bash
# =============================================================================
# @Project : Semantic SLAM Evaluation Framework
# @Desc    : Evaluation framework for comparing Visual vs LIDAR SLAM 
#            algorithms (ORB-SLAM3, RTAB-Map, Cartographer) augmented 
#            with zero-shot semantic segmentation (SAM2 / DeepLabV3).
# =============================================================================
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

# Clean the TUM trajectory file to ensure evo parser compatibility.
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

BASENAME=$(basename "$ESTIMATED" .txt)
RESULTS_DIR="/ros2_ws/src/results"
mkdir -p "$RESULTS_DIR"

echo "Saving evaluation results to $RESULTS_DIR"

evo_ape tum "$GROUNDTRUTH" "$ESTIMATED" -a --plot_mode=xyz --t_max_diff 0.1 --save_plot "${RESULTS_DIR}/${BASENAME}_ape.png" --save_results "${RESULTS_DIR}/${BASENAME}_ape.zip"

echo ""
echo "=========================================================="
echo " Executing Relative Pose Error (RPE) Analysis"
echo "=========================================================="

evo_rpe tum "$GROUNDTRUTH" "$ESTIMATED" -a --plot_mode=xyz --t_max_diff 0.1 --save_plot "${RESULTS_DIR}/${BASENAME}_rpe.png" --save_results "${RESULTS_DIR}/${BASENAME}_rpe.zip"
