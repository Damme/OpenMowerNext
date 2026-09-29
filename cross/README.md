# Cross compiling for the Pi (arm64)

Builds `open_mower_next` for the Pi's Ubuntu 24.04 arm64 + Jazzy rootfs on an x86 host, without qemu.
Only the compiler is cross (`aarch64-linux-gnu-g++` 13). cmake, colcon and the rosidl generators run
natively in an x86 Ubuntu 24.04 container.

| step | script | when | time |
|---|---|---|---|
| 1 | `1-build-rootfs.sh [image]` | package.xml dependencies changed | ~40 min (qemu) |
| 2 | `2-make-sysroot.sh [image] [dir]` | after step 1 | ~20 s |
| 3 | `3-build.sh [colcon args]` | every build | ~3 min clean, ~30 s clean with warm ccache |

```bash
cross/1-build-rootfs.sh                 # -> image om2-rootfs:arm64
cross/2-make-sysroot.sh                 # -> /opt/sysroot_jazzy_arm64
cross/3-build.sh                        # -> /opt/om2_xws/install/open_mower_next
```

The same rootfs image is the runtime rootfs on the Pi (`/opt/om2/rootfs`) and the sysroot. Keep them in sync.
To check, compare `var/lib/dpkg/status` in both: the md5 must match.

## How it works

- **Sysroot**: `docker export` of the arm64 rootfs image. `fix_symlinks.py` makes absolute symlinks
  relative (e.g. `libblas.so.3 -> /etc/alternatives/...`) so they resolve inside the sysroot.
- **Absolute paths in cmake configs**: installed ROS/system cmake configs contain paths such as
  `/opt/ros/jazzy/lib/librclcpp.so` or `/usr/include/eigen3`. `3-build.sh` mounts the sysroot's
  `/opt/ros/jazzy`, `/usr/include` and `/usr/lib/aarch64-linux-gnu` at those same paths in the container,
  so nothing has to be patched.
- **Toolchain**: `toolchain-aarch64.cmake` sets the sysroot, restricts package/library/include lookup to it,
  and adds `-rpath-link` for transitive shared-library dependencies.
- The workspace is mounted at `/ws`, as on the Pi, so the install tree and its setup files can be copied
  into `rootfs/ws/install/` unchanged. The source is mounted read-only.

Env overrides for `3-build.sh`: `SYSROOT`, `XWS` (workspace), `JOBS` (8), `BUILD_TYPE` (RelWithDebInfo).
