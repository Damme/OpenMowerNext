# Cross toolchain: x86 Ubuntu 24.04 container -> Pi rootfs (Ubuntu 24.04 arm64 + ROS 2 Jazzy).
# The sysroot is mounted at /sysroot; 3-build.sh also mounts its /opt/ros/jazzy, /usr/include and
# /usr/lib/aarch64-linux-gnu at the same paths, so absolute paths baked into the installed cmake configs
# resolve to arm64 files without sed-patching them.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(CMAKE_SYSROOT /sysroot)
set(CMAKE_FIND_ROOT_PATH /sysroot)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Transitive shared-lib deps (DT_NEEDED) of ROS libs are outside the default linker search path.
set(_rpath_link "-Wl,-rpath-link,/sysroot/opt/ros/jazzy/lib -Wl,-rpath-link,/sysroot/usr/lib/aarch64-linux-gnu")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${_rpath_link}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${_rpath_link}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${_rpath_link}")

set(ENV{PKG_CONFIG_SYSROOT_DIR} /sysroot)
set(ENV{PKG_CONFIG_LIBDIR} /sysroot/usr/lib/aarch64-linux-gnu/pkgconfig:/sysroot/usr/share/pkgconfig)
