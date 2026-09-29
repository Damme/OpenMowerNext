# x86 build image for cross compiling open_mower_next to arm64.
# Same distro as the target rootfs, so the cross gcc 13 / glibc 2.39 / python 3.12 match the Pi.
# Host-side tools (cmake, colcon, python rosidl generators) run natively, only the compiler is cross.
FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      g++-aarch64-linux-gnu gcc-aarch64-linux-gnu ccache python3-numpy \
    && rm -rf /var/lib/apt/lists/*
