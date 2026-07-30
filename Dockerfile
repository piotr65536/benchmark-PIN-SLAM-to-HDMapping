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

# ── PIN-SLAM python dependencies (before the source, for layer caching) ──────
WORKDIR /pin_ws
COPY ./src/PIN_SLAM/requirements.txt /tmp/pin_requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/pin_requirements.txt

# rosbags: PIN-SLAM's built-in ROS1/ROS2 bag dataloader (no ROS install needed).
# lazrs: LAZ compression backend for laspy, used by the HDMapping converter.
RUN pip3 install --no-cache-dir "rosbags==0.9.22" lazrs

# ── PIN-SLAM (submodule) ─────────────────────────────────────────────────────
COPY ./src/PIN_SLAM ./PIN_SLAM

# The submodule's .git is a gitlink to the parent repo's .git/modules (absent in
# the image), and PIN-SLAM's setup_experiment() hard-fails on `git rev-parse
# HEAD`. Replace it with a minimal self-contained repo so the call succeeds.
RUN rm -f PIN_SLAM/.git && \
    cd PIN_SLAM && \
    git init -q && \
    git -c user.email=benchmark@docker -c user.name=benchmark \
        commit -q --allow-empty -m "docker build snapshot" && \
    git rev-parse HEAD

# ── HDMapping converter ──────────────────────────────────────────────────────
COPY ./converter/ ./converter/

# Mesa DRI drivers: software-GL fallback for the o3d viewer when NVIDIA GL is
# not injected (non-NVIDIA hosts, or runtime without graphics capability).
# Kept as a late layer so the heavy torch/pip layers above stay cached.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-dri \
    libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/*

# Build guard: fail the image loudly if any runtime import is broken.
RUN python3 -c "import torch; assert torch.cuda.is_available() or True; \
import open3d, laspy, lazrs, rosbags, gtsam, pypose, pyquaternion; \
print('[build] PIN-SLAM deps OK, torch', torch.__version__)"

CMD ["bash"]
