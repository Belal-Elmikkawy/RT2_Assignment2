# Technical Progress Report: SAM-Enhanced 3D Semantic SLAM

**Project Title:** SAM-Enhanced 3D Semantic SLAM  
**Report Type:** Technical Progress & Node Architecture Report  

---

## 1. Executive Summary & Assignment Overview

This report details the architectural design and current implementation status of the **SAM-Enhanced 3D Semantic SLAM** project. The primary objective of this assignment is to integrate the Segment Anything Model (SAM/SAM2) into modern 3D SLAM pipelines to produce semantically labeled 3D point cloud maps in real-time. 

**Core Project Requirements:**
1. Setup and evaluate three distinct SLAM systems (ORB-SLAM3, RTAB-Map, and Cartographer).
2. Implement a SAM-based perception module for real-time semantic segmentation of RGB keyframes.
3. Develop a custom fusion node to project 2D SAM masks onto 3D point clouds using synchronized depth and odometry data.
4. Evaluate mapping quality (ATE, RPE, semantic consistency) and runtime performance (FPS).
5. Validate the system both in simulation (using Gazebo with a G1 Humanoid robot) and through real-world dataset replay (TUM RGB-D).
6. Compare the SAM-enhanced approach against a baseline semantic SLAM (e.g., DeepLabV3).

---

## 2. Current Implementation Status

Significant progress has been made in establishing the core ROS 2 node architecture and achieving synchronized semantic point cloud fusion. 

### ✅ Completed Milestones
- **Dataset Integration:** Developed `dataset_publisher_node` for reliable TUM RGB-D dataset replay.
- **Semantic Perception:** Implemented `sam2_perception_node` for real-time instance segmentation utilizing SAM2 with GPU acceleration.
- **SLAM Backend Integration:** 
  - Successfully integrated `rtabmap_odom` for RGB-D visual odometry.
  - Successfully integrated `orbslam3_node` for alternative visual odometry.
  - Successfully integrated `cartographer_node` and `cartographer_grid_node` for LiDAR-based 2D/3D SLAM.
- **Coordinate Transformations:** Implemented `tf_to_odom_node` to bridge Cartographer TF data to standard Odometry messages.
- **Semantic Fusion:** Developed the core `semantic_fusion_node` which successfully synchronizes SAM2 masks, depth images, and SLAM odometry to generate a colored 3D semantic point cloud.

### ⏳ Pending Tasks (Next Steps)
- Complete full 3D mapping integration with RTAB-Map's `map_server`.
- Finalize benchmark evaluation metrics (ATE, RPE computation).
- Implement the DeepLabV3 baseline for comparative analysis.
- Develop real-time performance monitoring nodes and dashboard.

---

## 3. System Architecture & Pipeline Modes

The system architecture is designed to be modular and supports two primary operational modes: Simulation and Dataset Replay.

### 3.1 Mode 1: Simulation Pipeline (`semantic_pipeline.launch.py`)

In this mode, the system operates within a Gazebo simulation environment featuring the Unitree G1 Humanoid Robot equipped with an RGB-D camera plugin.

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                          GAZEBO SIM ENVIRONMENT                              │
│  [G1 Humanoid Robot + RGB-D Camera Plugin] → /camera/color/image_raw        │
│                                            → /camera/depth/image_raw         │
│                                            → /camera/color/camera_info      │
└─────────────────────────────────────────────────────────────────────────────┘
              │                                        │
              ├──────────────────────┬────────────────┴─────────────────┐
              ▼                      ▼                                  ▼
    ┌─────────────────┐    ┌──────────────────┐            ┌────────────────────┐
    │   SAM2 Perception│    │ RTAB-Map Odometry │            │ Cartographer SLAM  │
    │ sam2_perception  │    │  rgbd_odometry    │            │ cartographer_node  │
    └────────┬─────────┘    └────────┬──────────┘            └────────┬───────────┘
             │                       │                               │
             │ /sam2/semantic_mask   │ /slam/odom (camera pose)      │ /map (occupancy grid)
             │                       │ /odom → /base_link (TF)       │ /map → /base_link (TF)
             │                       │                               │
             └───────────┬───────────┴───────────────────────────────┤
                         │                                           │
                         ▼                                           ▼
          ┌──────────────────────┐                      ┌─────────────────────┐
          │ Semantic Fusion Node │◄─────────────────────│ TF-to-Odom Bridge   │
          │ semantic_fusion_node │                      │   tf_to_odom        │
          └──────────┬───────────┘                      └─────────────────────┘
                     │
                     │ /semantic_map (PointCloud2 with RGB labels)
                     ▼
          ┌──────────────────────┐
          │      RViz2           │
          │   (Visualization)    │
          └──────────────────────┘
```

### 3.2 Mode 2: Dataset Replay Pipeline (`dataset_pipeline.launch.py`)

This mode replaces the simulation with a node that replays the TUM RGB-D dataset, allowing for repeatable benchmarking against ground truth data.

```text
┌──────────────────────────────────────────────────────────────┐
│      TUM RGB-D Dataset Replay (On Disk)                      │
│      dataset_publisher_node → /camera/color/image_raw        │
│                            → /camera/depth/image_raw         │
│                            → /camera/color/camera_info       │
└──────────────────────────────────────────────────────────────┘
              │                                    
              ├──────────┬──────────────┬──────────────┐
              ▼          ▼              ▼              ▼
    ┌─────────────────┐ ┌─────────────┐ ┌────────────────┐
    │ SAM2 Perception │ │ RTAB-Map    │ │  ORB-SLAM3     │
    │ sam2_perception │ │ Odometry    │ │  rgbd          │
    └────────┬─────────┘ └─────┬───────┘ └────────┬───────┘
             │                 │                  │
             │ /sam2/          │ /slam/odom       │ /camera_pose_tf
             │ semantic_mask   │ (visual odom)    │ (camera pose)
             │                 │                  │
             └────────┬────────┴──────────────────┤
                      │                           │
                      ▼                           │
         ┌──────────────────────┐                 │
         │ Semantic Fusion Node │◄────────────────┘
         │ semantic_fusion_node │
         └──────────┬───────────┘
                    │
                    │ /semantic_map (PointCloud2 with instance labels)
                    ▼
         ┌──────────────────────┐
         │      RViz2           │
         │ (Visualization)      │
         └──────────────────────┘
```

---

## 4. Detailed Node Descriptions

The architecture consists of several specialized ROS 2 nodes, each handling a specific part of the semantic SLAM pipeline.

### 4.1 Data Source Nodes
* **`dataset_publisher_node`**: Replays TUM RGB-D or ScanNet datasets. It publishes synchronized RGB (`/camera/color/image_raw`), depth (`/camera/depth/image_raw`), and camera intrinsics (`/camera/color/camera_info`) at a configurable frame rate to simulate a real-time sensor stream.

### 4.2 Perception Nodes
* **`sam2_perception_node`**: The core AI perception module. It subscribes to the RGB stream and runs the Meta SAM2 model (GPU-accelerated) to perform real-time instance segmentation. To maintain real-time performance, it operates on a keyframe interval (e.g., processing every 3rd frame) and publishes `/sam2/semantic_mask` containing per-pixel instance labels.

### 4.3 SLAM Backend Nodes
* **`rgbd_odometry` (RTAB-Map)**: Computes visual odometry from the RGB-D stream. It publishes the incremental camera pose to `/slam/odom` and broadcasts the corresponding TF tree.
* **`orbslam3_node`**: An alternative monolithic visual SLAM backend that uses ORB features for robust tracking, loop closure, and bundle adjustment. It publishes the absolute `/camera_pose`.
* **`cartographer_node`**: Provides robust LiDAR-based SLAM. Used primarily in the simulation environment to build 2D/3D occupancy grids (`/map`) and establish global localization.
* **`tf_to_odom_node`**: A custom utility node that bridges Cartographer’s TF output (`map` → `base_link`) into standard `nav_msgs/Odometry` messages for backend-agnostic fusion.

### 4.4 Fusion & Visualization Nodes
* **`semantic_fusion_node`**: The most critical custom node in the pipeline. It uses a `message_filters::Synchronizer` with an approximate time policy to align the decoupled outputs: the SAM2 semantic mask, the raw depth map, and the SLAM odometry. It deprojects the 2D masked pixels into 3D space, transforms them into the global world frame using the odometry, and accumulates them into a voxel-downsampled colored `PointCloud2` (`/semantic_map`).

---

## 5. Synchronization & Technical Challenges Addressed

A major technical achievement in this project was resolving the synchronization bottlenecks between the highly heterogeneous node frequencies.

* **The Challenge:** The camera captures data at 30 Hz. SLAM odometry updates at 20-30 Hz. However, SAM2 inference is computationally heavy, yielding a mask at ~10 Hz (decimated by `keyframe_interval=3`).
* **The Solution:** Implemented an `ApproximateTime` sync policy in the `semantic_fusion_node` with a carefully tuned queue size (20 frames). This ensures that when a SAM2 mask is successfully generated, it is correctly paired with the historically closest depth map and SLAM pose, preventing data dropping and TF lookup timeouts.

---

## 6. Performance Characteristics

Preliminary runtime profiling indicates that the pipeline is capable of near real-time execution on a CUDA-enabled machine.

| Pipeline Stage | Latency Component | Est. Duration (ms) |
| :--- | :--- | :--- |
| **Input** | Camera Capture (@ 30Hz) | ~33 ms |
| **Perception** | SAM2 Inference (GPU, FP16) | 200 - 500 ms (per keyframe) |
| **Tracking** | SLAM Odometry (RTAB/ORB) | 20 - 50 ms |
| **Mapping** | 3D Semantic Point Cloud Fusion | 10 - 20 ms |
| **Total** | **End-to-End Latency** | **~300 - 600 ms** |

**Throughput Optimization:** Total bandwidth for raw images, masks, and point clouds approaches ~170 Mbps. To manage this, voxel grid filtering (leaf size: 0.05m) is applied within the semantic fusion node before publishing to RViz.

---

## 7. Conclusion

The foundational architecture for the SAM-Enhanced 3D Semantic SLAM pipeline is complete and functional. The modular design allows for seamless switching between simulation and real-world datasets, as well as interchanging SLAM backends (ORB-SLAM3, RTAB-Map, Cartographer). Moving forward, the focus will shift towards quantitative evaluation (ATE/RPE metrics) and baseline comparisons to finalize the assignment requirements.
