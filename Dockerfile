FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# ── Base tools + libraries needed by open3d / opencv / the o3d GUI ───────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    git \
    ca-certificates \
    libgl1 \
    libegl1 \
    libgomp1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libx11-6 \
    libxi6 \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    && rm -rf /var/lib/apt/lists/*

# ── PyTorch (CUDA 11.8) ──────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir torch==2.1.2 --index-url https://download.pytorch.org/whl/cu118

# ── PIN-SLAM (submodule) + its python dependencies ───────────────────────────
WORKDIR /pin_ws
COPY ./src/PIN_SLAM ./PIN_SLAM

RUN pip3 install --no-cache-dir -r PIN_SLAM/requirements.txt

# rosbags: PIN-SLAM's built-in ROS1/ROS2 bag dataloader (no ROS install needed).
# lazrs: LAZ compression backend for laspy, used by the HDMapping converter.
RUN pip3 install --no-cache-dir "rosbags==0.9.22" lazrs

# ── HDMapping converter ──────────────────────────────────────────────────────
COPY ./converter/ ./converter/

# Build guard: fail the image loudly if any runtime import is broken.
RUN python3 -c "import torch; assert torch.cuda.is_available() or True; \
import open3d, laspy, lazrs, rosbags, gtsam, pypose, pyquaternion; \
print('[build] PIN-SLAM deps OK, torch', torch.__version__)"

CMD ["bash"]
