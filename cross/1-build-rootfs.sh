#!/bin/bash
# Build the arm64 target rootfs image (Ubuntu 24.04 + Jazzy + rosdep deps) under qemu.
# Only needed when package.xml dependencies change. ~40 min.
# Usage: cross/1-build-rootfs.sh [image tag]   (default om2-rootfs:arm64)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
IMAGE=${1:-om2-rootfs:arm64}

# the build context only needs package.xml (rosdep keys)
CTX=$(mktemp -d)
trap 'rm -rf "$CTX"' EXIT
cp "$HERE/../package.xml" "$CTX/"
docker buildx build --platform linux/arm64 --load -t "$IMAGE" -f "$HERE/Dockerfile.rootfs" "$CTX"
echo "Built $IMAGE. Next: cross/2-make-sysroot.sh $IMAGE"
