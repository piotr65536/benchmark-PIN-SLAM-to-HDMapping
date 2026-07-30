#!/bin/bash
# Runs PIN-SLAM on a bag and converts the result to an HDMapping session.
#
# Unlike the ROS-based benchmarks (roscore/tmux/rosbag record), PIN-SLAM runs
# OFFLINE: its built-in dataloader reads the ROS1 .bag file (or ROS2 bag
# directory) directly and writes TUM-format pose files stamped with sensor
# time. The converter then rebuilds the dense world-frame cloud from the input
# bag + poses and writes the HDMapping session. Live progress is shown in
# PIN-SLAM's own 3D viewer (instead of RViz).
#
# Usage:
#   ./docker_session_run-pin-slam.sh <input.bag | ros2_bag_dir> <output_dir>
#
# Environment variables:
#   CONFIG     - PIN-SLAM config inside the repo (default: config/lidar_slam/run.yaml)
#   TOPIC      - point cloud topic in the bag (default: /livox/pointcloud)
#   VIS        - 1 = show PIN-SLAM 3D viewer (default: 1; auto-off without DISPLAY)
#   CPU_ONLY   - 1 = run on CPU (very slow; default 0 = CUDA GPU)
#   POINT_SKIP - converter keeps every Nth point (default 1 = all)

set -e

IMAGE_NAME='pin-slam_cuda'
OUTPUT_NAME='output_hdmapping-PIN-SLAM'

CONFIG="${CONFIG:-config/lidar_slam/run.yaml}"
TOPIC="${TOPIC:-/livox/pointcloud}"
VIS="${VIS:-1}"
CPU_ONLY="${CPU_ONLY:-0}"
POINT_SKIP="${POINT_SKIP:-1}"

if [[ $# -lt 2 ]]; then
  if command -v zenity >/dev/null 2>&1; then
    DATASET_HOST_PATH=$(zenity --file-selection --title="Select input bag (.bag file or ROS2 bag directory)")
    OUTPUT_HOST_DIR=$(zenity --file-selection --directory --title="Select output directory")
  else
    echo "Usage: $0 <input.bag | ros2_bag_dir> <output_dir>"
    echo ""
    echo "  CONFIG     - PIN-SLAM config (default: config/lidar_slam/run.yaml)"
    echo "  TOPIC      - point cloud topic (default: /livox/pointcloud)"
    echo "  VIS        - 1 = live 3D viewer (default: 1)"
    echo "  CPU_ONLY   - 1 = CPU only, very slow (default: 0)"
    echo "  POINT_SKIP - keep every Nth point in converter (default: 1)"
    exit 1
  fi
else
  DATASET_HOST_PATH=$(realpath "$1")
  OUTPUT_HOST_DIR=$(realpath "$2")
fi

if [[ ! -e "$DATASET_HOST_PATH" ]]; then
  echo "Error: input $DATASET_HOST_PATH does not exist"
  exit 1
fi
mkdir -p "$OUTPUT_HOST_DIR"

DATASET_DIR=$(dirname "$DATASET_HOST_PATH")
DATASET_BASE=$(basename "$DATASET_HOST_PATH")

# GPU: prefer the nvidia runtime (works in both legacy and CDI-mode toolkit
# configs, where the --gpus hook path is rejected); fall back to --gpus all,
# then to CPU (PIN-SLAM on CPU is very slow — expect it only for tiny bags).
if [[ "$CPU_ONLY" == "1" ]]; then
  GPU_ARGS=""
elif docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia'; then
  # capabilities=all also injects the NVIDIA GL libraries, so the o3d viewer
  # can render hardware-accelerated inside the container.
  GPU_ARGS="--runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all"
elif docker run --rm --gpus all ubuntu:22.04 true >/dev/null 2>&1; then
  GPU_ARGS="--gpus all"
else
  GPU_ARGS=""
  CPU_ONLY=1
  echo "WARNING: no working NVIDIA docker runtime detected — running on CPU (very slow)."
fi

# Viewer needs X11 (same recipe as the RViz-based benchmarks: host network +
# X socket mount + xhost; additionally pass the X authority file if present).
XAUTH_ARGS=""
if [[ "$VIS" == "1" && -n "$DISPLAY" ]]; then
  VIS_FLAG="-v"
  xhost +local:docker >/dev/null 2>&1 || true
  if [[ -n "$XAUTHORITY" && -f "$XAUTHORITY" ]]; then
    XAUTH_ARGS="-e XAUTHORITY=/tmp/.host_xauth -v $XAUTHORITY:/tmp/.host_xauth:ro"
  fi
else
  VIS_FLAG=""
fi

echo "Input bag       : $DATASET_HOST_PATH"
echo "Output dir      : $OUTPUT_HOST_DIR/$OUTPUT_NAME"
echo "PIN-SLAM config : $CONFIG"
echo "Cloud topic     : $TOPIC"
echo "Viewer          : ${VIS_FLAG:-off}"
echo "CPU only        : $CPU_ONLY"

docker run -it --rm \
  --network host \
  $GPU_ARGS \
  $XAUTH_ARGS \
  -e DISPLAY="$DISPLAY" \
  -e XDG_RUNTIME_DIR=/tmp/xdg \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DATASET_DIR":/data \
  -v "$OUTPUT_HOST_DIR":/output \
  "$IMAGE_NAME" bash -c "
set -e
cd /pin_ws/PIN_SLAM

CPU_FLAG=''
if [[ '$CPU_ONLY' == '1' ]]; then CPU_FLAG='-c'; fi

echo '[pin-slam] running SLAM (this processes the whole bag offline)...'
python3 pin_slam.py '$CONFIG' rosbag '$TOPIC' -d \
    -i '/data/$DATASET_BASE' \
    -o /tmp/pin_experiments \
    \$CPU_FLAG $VIS_FLAG

RUN_DIR=\$(ls -td /tmp/pin_experiments/*/ | head -1)
echo \"[pin-slam] done. Run dir: \$RUN_DIR\"

echo '[converter] building HDMapping session...'
python3 /pin_ws/converter/pinslam_to_hdmapping.py \
    --bag '/data/$DATASET_BASE' \
    --run-dir \"\$RUN_DIR\" \
    --topic '$TOPIC' \
    --out '/output/$OUTPUT_NAME' \
    --point-skip '$POINT_SKIP'

# Keep the raw PIN-SLAM pose files next to the session (useful for evo APE).
cp -v \"\$RUN_DIR\"/*_tum.txt '/output/$OUTPUT_NAME/' 2>/dev/null || true
chmod -R a+rw '/output/$OUTPUT_NAME'
"

echo "=== DONE === Results in: $OUTPUT_HOST_DIR/$OUTPUT_NAME"
