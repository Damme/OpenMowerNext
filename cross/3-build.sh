#!/bin/bash
# Cross compile open_mower_next for the Pi (arm64) in an x86 container.
# Usage: cross/3-build.sh [extra colcon build args]      e.g. cross/3-build.sh --cmake-clean-cache
# Env:   SYSROOT (default /opt/sysroot_jazzy_arm64), XWS workspace (default /opt/om2_xws),
#        JOBS (default 8), BUILD_TYPE (default RelWithDebInfo)
# Output: $XWS/install/open_mower_next, built for the prefix /ws/install like the Pi's rootfs expects.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$(cd "$HERE/.." && pwd)
SYSROOT=${SYSROOT:-/opt/sysroot_jazzy_arm64}
XWS=${XWS:-/opt/om2_xws}
JOBS=${JOBS:-8}
BUILD_TYPE=${BUILD_TYPE:-RelWithDebInfo}
IMAGE=om2-cross:jazzy

[ -d "$SYSROOT/opt/ros/jazzy" ] || { echo "No sysroot at $SYSROOT, run cross/2-make-sysroot.sh first"; exit 1; }
docker image inspect "$IMAGE" >/dev/null 2>&1 || docker build -t "$IMAGE" -f "$HERE/Dockerfile" "$HERE"
mkdir -p "$XWS/src/open_mower_next" "$XWS/ccache"

# The sysroot's /opt/ros/jazzy, /usr/include and /usr/lib/aarch64-linux-gnu are also mounted at their real
# paths: installed cmake configs contain absolute paths (e.g. /opt/ros/jazzy/lib/librclcpp.so). The x86 image
# does not need its own copies of those (python generators are pure python, the compiler is the cross gcc).
# CMakeLists.txt hard-codes /usr/share/cmake/geographiclib as module path.
docker run --rm --network none -u "$(id -u):$(id -g)" -e HOME=/tmp \
  -e CCACHE_DIR=/ws/ccache -e MAKEFLAGS="-j$JOBS" \
  -v "$SYSROOT":/sysroot:ro \
  -v "$SYSROOT/opt/ros/jazzy":/opt/ros/jazzy:ro \
  -v "$SYSROOT/usr/include":/usr/include:ro \
  -v "$SYSROOT/usr/lib/aarch64-linux-gnu":/usr/lib/aarch64-linux-gnu:ro \
  -v "$SYSROOT/usr/share/cmake/geographiclib":/usr/share/cmake/geographiclib:ro \
  -v "$HERE/toolchain-aarch64.cmake":/toolchain.cmake:ro \
  -v "$XWS":/ws \
  -v "$SRC":/ws/src/open_mower_next:ro \
  -w /ws "$IMAGE" bash -c '
    source /opt/ros/jazzy/setup.bash
    colcon build --event-handlers console_direct- --packages-select open_mower_next "$@" \
      --cmake-args -DCMAKE_TOOLCHAIN_FILE=/toolchain.cmake -DCMAKE_BUILD_TYPE='"$BUILD_TYPE"' \
        -DBUILD_TESTING=OFF -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
  ' bash "$@"

echo
echo "Built $XWS/install/open_mower_next (arm64). Deploy, e.g.:"
echo "  rsync -a --delete $XWS/install/open_mower_next/ damme@10.99.99.99:/opt/om2/rootfs/ws/install/open_mower_next/"
