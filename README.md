# benchmark-PIN-SLAM-to-HDMapping

Runs the [PIN-SLAM](https://github.com/PRBonn/PIN_SLAM) LiDAR SLAM algorithm on
a ROS 1 / ROS 2 bag and converts the output to an
[HDMapping](https://github.com/MapsHD/HDMapping) session.

PIN-SLAM (*LiDAR SLAM Using a Point-Based Implicit Neural Representation for
Achieving Global Map Consistency*, Y. Pan et al., IPB Bonn) is a LiDAR-only
SLAM: it builds an implicit neural map from point clouds (no IMU used) and
keeps the map globally consistent with loop closures + pose graph
optimization.

Unlike the ROS-based benchmarks in this family, PIN-SLAM runs **offline**:
its built-in dataloader reads the bag directly (ROS 1 `.bag` file or ROS 2 bag
directory — no conversion needed), and live progress is shown in PIN-SLAM's
own 3D viewer instead of RViz.

## Prerequisites

- Docker
- **NVIDIA GPU + [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)**
  (PIN-SLAM is PyTorch/CUDA based; CPU-only mode exists but is very slow)
- A bag containing a `sensor_msgs/PointCloud2` topic

## Step 1 — Clone with submodules

```bash
git clone https://github.com/MapsHD/benchmark-PIN-SLAM-to-HDMapping.git --recursive
cd benchmark-PIN-SLAM-to-HDMapping
```

## Step 2 — Build the Docker image

```bash
docker build -t pin-slam_cuda .
```

This installs:
- CUDA 11.8 runtime (Ubuntu 22.04) + PyTorch 2.1
- PIN-SLAM (from the `src/PIN_SLAM` submodule) and its Python dependencies
- `rosbags` (bag reading without ROS) and `laspy`/`lazrs` (LAZ writing)

## Step 3 — Run the pipeline

```bash
chmod +x docker_session_run-pin-slam.sh
TOPIC=/your/pointcloud_topic ./docker_session_run-pin-slam.sh /path/to/input.bag /path/to/output/dir
```

Environment variables:

| Variable | Default | Meaning |
|----------|---------|---------|
| `CONFIG` | `config/lidar_slam/run.yaml` | PIN-SLAM config file (inside the PIN_SLAM repo) |
| `TOPIC` | `/livox/pointcloud` | point cloud topic in the bag |
| `VIS` | `1` | show PIN-SLAM's live 3D viewer (auto-off without `DISPLAY`) |
| `VIEWER_HOLD` | `30` | seconds the viewer stays open after SLAM finishes, then auto-close + convert |
| `CPU_ONLY` | `0` | force CPU (very slow) |
| `POINT_SKIP` | `1` | converter keeps every Nth point |
| `CONVERT_RUN_DIR` | – | existing `pin_experiments` run dir (container path): skip SLAM, only convert |

**What happens:**

1. `pin_slam.py` processes the whole bag offline (with the live viewer if
   `VIS=1`) and writes TUM-format pose files stamped with **sensor time**.
2. The converter rebuilds the **dense** world-frame cloud by transforming every
   input scan with its PIN-SLAM pose (loop-closure-corrected `slam_poses` are
   preferred over `odom_poses`), chunks it, and writes the HDMapping session.

## Step 4 — Open in HDMapping

Output files appear in `<output_dir>/output_hdmapping-PIN-SLAM/`:

```
lio_initial_poses.reg
poses.reg
scan_lio_0.laz
...
session.json
trajectory_lio_0.csv
...
odom_poses_tum.txt        (raw PIN-SLAM trajectory, TUM format — handy for evo)
slam_poses_tum.txt        (present when loop closures/PGO ran)
```

Open `session.json` with the
[multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping)
application.

## Contact

januszbedkowski@gmail.com
