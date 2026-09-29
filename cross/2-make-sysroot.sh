#!/bin/bash
# Unpack an arm64 rootfs image into the cross-compile sysroot and make its symlinks relative.
# The sysroot must match the Pi's /opt/om2/rootfs (compare /var/lib/dpkg/status to check).
# Usage: cross/2-make-sysroot.sh [image] [sysroot dir]
#   defaults: om2-rootfs:arm64, $SYSROOT or /opt/sysroot_jazzy_arm64
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
IMAGE=${1:-om2-rootfs:arm64}
SYSROOT=${2:-${SYSROOT:-/opt/sysroot_jazzy_arm64}}

case "$SYSROOT" in
  /opt/sysroot_*|/tmp/*) ;;
  *) echo "Refusing to wipe $SYSROOT (expected /opt/sysroot_* or /tmp/*)"; exit 1 ;;
esac
rm -rf "$SYSROOT"
mkdir -p "$SYSROOT"
cid=$(docker create --platform linux/arm64 "$IMAGE" /bin/true)
trap 'docker rm "$cid" >/dev/null' EXIT
echo "Exporting $IMAGE to $SYSROOT ..."
# /ws: an image built the old way contains a qemu build of our package; it must not shadow the cross build.
docker export "$cid" | tar -x -C "$SYSROOT" --anchored --exclude='dev/*' --exclude='ws'

# e.g. libblas.so.3 -> /etc/alternatives/... would otherwise resolve on the build host
python3 "$HERE/fix_symlinks.py" "$SYSROOT"
du -sh "$SYSROOT"
