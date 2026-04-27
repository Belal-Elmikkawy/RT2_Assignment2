# Subgroup I1: SAM‑Enhanced 3D Semantic SLAM


## Assignment 1: Real‑Time Semantic Mapping via Foundation Model Segmentation (EXPERIMENT + SIMULATION)

What to do: Integrate SAM/SAM2 segmentation into a 3D SLAM pipeline to produce semantically labeled 3D maps in real‑time, and benchmark multiple SLAM backends to evaluate which architecture best supports foundation‑model‑based semantic integration
1) Set up and test three SLAM systems (ORB‑SLAM3, RTAB‑Map, Cartographer) on a common indoor dataset (TUM RGB‑D, ScanNet, ...)
2) Implement a SAM‑based segmentation module that processes RGB keyframes and produces per‑pixel semantic masks
3) Develop a fusion node that projects SAM masks onto the 3D point cloud / occupancy map produced by each SLAM backend
4) Evaluate mapping quality: geometric accuracy (ATE, RPE), semantic label consistency across frames, and real‑time performance (FPS, latency)
5) Test in simulation (Gazebo + RGB‑D camera) and optionally on a handheld sensor
6) Compare against a baseline semantic SLAM that uses classical segmentation (DeepLabV3)

Software needed: ROS2 Humble, ORB‑SLAM3, RTAB‑Map, Cartographer, SAM / SAM2 (PyTorch), Open3D, Gazebo, OpenCV
Research needed: Visual SLAM benchmarking, SAM architecture and zero‑shot capabilities, 3D semantic mapping methods, point cloud labeling techniques, real‑time inference optimization
Deliverables: ROS2 integration package for SAM + SLAM, benchmark dataset with ground truth, quantitative comparison report across SLAM backends, semantic 3D map visualizations

# Starting point
Understand these repos:
- https://github.com/UZ-SLAMLab/ORB_SLAM3
- https://github.com/introlab/rtabmap
- https://github.com/cartographer-project/cartographer
- https://github.com/facebookresearch/sam2
- https://github.com/facebookresearch/sam3
- Decide the dataset, understand why one could be better than another one
