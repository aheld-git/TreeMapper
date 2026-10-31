# tm_sim_ros: the ROS 1 Noetic half of the Isaac-Sim-only closed loop.
#
# Isaac Sim 5.1 (ROS 2 Humble, its own container) <-> ros1_bridge <-> this
# container (Noetic: perception, TreeMapper, navigator).
#
# The base image already provides Ubuntu 20.04, /opt/ros/noetic, ROS 2 Humble
# built from source at /ros2_humble/install and a built ros1_bridge
# (parameter_bridge). Isaac is Humble too, so there is no Foxy anywhere. This
# file only adds what the base lacks: onnxruntime-gpu plus the CUDA 12 / cuDNN 9
# runtime libs it needs (the base ships no CUDA), and a clean entrypoint.
#
#   docker build -t tm_sim_ros:latest sim/bridge
# The base is a LOCAL image (built by the TreeHoppingNavigation project, not pulled from a registry),
# so `latest` is only as stable as that machine's copy. The spike was run on
# sha256:4d87252ea3a4fc8cd225fccc026b1026ba7e02bd9c49416a5de15a5943ed97f4
# (`docker inspect -f '{{.Id}}' treehoppingnavigation-bridge:latest`); check it when results differ.
FROM treehoppingnavigation-bridge:latest

ENV DEBIAN_FRONTEND=noninteractive

# Already in the base (checked at build time, installed only when missing):
# Noetic cv_bridge / tf2_ros / image_transport / rosbag, python3-opencv, numpy,
# PyYAML, libyaml-cpp-dev.
RUN set -eux; \
    missing=""; \
    for p in ros-noetic-cv-bridge ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
             ros-noetic-image-transport ros-noetic-rosbag ros-noetic-rospy \
             python3-opencv python3-numpy python3-yaml libyaml-cpp-dev; do \
        dpkg -s "$p" >/dev/null 2>&1 || missing="$missing $p"; \
    done; \
    if [ -n "$missing" ]; then apt-get update && apt-get install -y --no-install-recommends $missing && rm -rf /var/lib/apt/lists/*; fi

# onnxruntime-gpu: the last release with a cp38 wheel is 1.19.2 (CUDA 12.x,
# cuDNN 9). The host driver (580) is CUDA 13 capable, so a CUDA 12 userland runs.
# The CUDA libs come from the pip nvidia-* wheels (the base has no toolkit).
RUN python3 -m pip install --no-cache-dir pip==25.0.1 \
 && python3 -m pip install --no-cache-dir \
        onnxruntime-gpu==1.19.2 \
        nvidia-cuda-runtime-cu12==12.4.127 \
        nvidia-cuda-nvrtc-cu12==12.4.127 \
        nvidia-cublas-cu12==12.4.5.8 \
        nvidia-cufft-cu12==11.2.1.3 \
        nvidia-curand-cu12==10.3.5.147 \
        nvidia-nvjitlink-cu12==12.4.127 \
        nvidia-cudnn-cu12==9.1.0.70

# Make the pip-provided CUDA libs visible to every process (no LD_LIBRARY_PATH
# juggling next to the ROS setup scripts).
RUN set -eux; \
    sp=/usr/local/lib/python3.8/dist-packages/nvidia; \
    ls -d $sp/*/lib > /etc/ld.so.conf.d/tm-cuda-pip.conf; \
    ldconfig; \
    cat /etc/ld.so.conf.d/tm-cuda-pip.conf

# Expose the GPU compute libs when run with --gpus all.
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    ROS_DOMAIN_ID=77 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY entrypoint.sh /tm_entrypoint.sh
COPY bridge_topics.yaml /tm/bridge_topics.yaml
RUN chmod +x /tm_entrypoint.sh

# The base image's entrypoint waits for a running sim and launches the
# TreeHoppingNavigation stack; replace it. `docker run ... tm_sim_ros <cmd>` runs
# <cmd> with ROS 1 sourced; `tm_sim_ros bridge` starts roscore + parameter_bridge.
ENTRYPOINT ["/tm_entrypoint.sh"]
CMD ["bash"]
