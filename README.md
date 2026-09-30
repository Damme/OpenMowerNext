# OpenMowerNext – Worx fork

![OpenMowerNext](docs/public/logo128.png)

A ROS 2 Jazzy stack for **Worx Landroid** mowers, running on the mower's own **Raspberry Pi Zero 2 W**
(512 MB RAM). It is a fork of [jkaflik/OpenMowerNext](https://github.com/jkaflik/OpenMowerNext), the ROS 2 port of
[OpenMower](https://github.com/ClemensElflein/open_mower_ros), and it replaces an open_mower_ros (ROS 1) setup on
the same robot.

Development happens on the `worx` branch. The upstream parts (map server, map recorder, localization, Webots
simulation, OpenMower hardware) are still here and kept close to upstream.

## What the fork adds

| Area | Component | |
|---|---|---|
| Hardware | `worx_hardware` | ros2_control plugin for the Worx mainboard (JSON over SPI), with a firmware emulator for simulation |
| | `lsm6dsv_imu` | LSM6DSV IMU over I2C (FIFO burst reads) |
| | `ubx_gps` | u-blox NAV-PVT from a `str2str` TCP stream (RTK) |
| | `ina226` | INA226 battery current/voltage monitor |
| | `wheel_scale` | adaptive wheel odometry scale from RTK GPS |
| Mowing | `mower_logic` | mission as a BehaviorTree.CPP tree: undock, mow areas pass by pass, dock on low battery or rain, feel around obstacles after bumps |
| | `coverage_planner`, `coverage_server` | Boost.Geometry coverage planner (perimeter loops, serpentine or concentric fill) |
| Navigation | `ftc_controller` | Nav2 controller: FTC for mowing passes, Regulated Pure Pursuit for transits |
| | `lean_smac`, `costmap_layers` | Smac lattice planner that frees its search memory after each plan; `RimCostLayer` for the strip just outside recorded boundaries |
| | `docking_helper` | docking with a GPS gate, resets the pose while standing on the charger |
| Runtime | `om_launch`, `om_container` | python-free launcher and component container: the whole robot stack in four processes, ~210 MB |
| | `web_ui` | built-in control page (map, mow/home/stop, joystick, area editing, boundary recording) over HTTP + WebSocket |

The Worx firmware on the mainboard is a separate project:
[Damme/LandLord](https://github.com/Damme/LandLord).

## Repository layout

The repository root is a single `ament_cmake` package, `open_mower_next`.

```
src/<component>/     C++ nodes, one directory per component (headers next to sources)
src/msg|srv|action/  interfaces
cmake/*.cmake        build targets per component, included from CMakeLists.txt
config/              Nav2, controllers, behavior trees; config/hardware/worx*.yaml; config/launch/*.yaml (om_launch manifests)
launch/              python launch files (Webots sim, kinematic Worx sim, OpenMower hardware)
scripts/             om_generate.py (build-time URDF/controller generation), worx_* tools
cross/               arm64 cross build for the Pi (no qemu)
systemd/worx/        systemd units for the robot (stack, GPS serial, NTRIP)
test/                gtest unit tests, test/sim/ simulation scenarios
docs/                VitePress documentation
```

`src/lib/` is reserved for external dependencies imported from `custom_deps.yaml` (`make custom-deps`).

## Building

### For the robot (arm64 cross build)

Builds on an x86 host with Docker against a sysroot exported from the same Ubuntu 24.04 + Jazzy rootfs the Pi runs.
See [cross/README.md](cross/README.md).

```bash
cross/1-build-rootfs.sh     # only when package.xml dependencies change
cross/2-make-sysroot.sh
cross/3-build.sh            # -> /opt/om2_xws/install/open_mower_next
```

### On x86 (tests, simulation)

In a ROS 2 Jazzy environment (devcontainer or Docker):

```bash
make custom-deps deps
colcon build --packages-select open_mower_next --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select open_mower_next
```

## Running on the robot

The Pi's host OS is Ubuntu 20.04, so the stack runs rootless in an Ubuntu 24.04 arm64 rootfs under `/opt/om2`
(bubblewrap, host network, no Docker). The install tree goes to `/opt/om2/rootfs/ws/install/open_mower_next/`.
`systemd/worx/install.sh` installs three units:

- `om2-gps-serial` / `om2-gps-ntrip`: `str2str` serves the receiver's UBX stream on TCP 5015 and feeds it RTCM
  corrections (`/opt/om2/config/ntrip.env`).
- `om2-robot`: `om_launch` with `config/launch/worx.yaml`.

Required environment: `OM_MAP_PATH` (GeoJSON map), `OM_DATUM_LAT`, `OM_DATUM_LONG`. The manifest header lists the
optional ones (motors, GPS, battery thresholds, areas, web UI address). Motors start disabled unless
`OM_WORX_MOTORS_ENABLED=true`.

Maps recorded with open_mower_ros can be converted with `scripts/worx_map_to_geojson.py`.

The web UI listens on port 8090 of the robot's VPN address and only accepts the networks in `OM_WEB_ALLOW`.
There is no login – see [Web UI](docs/architecture/web-ui.md#access) before exposing it.

## Simulation

The kinematic Worx simulation runs the real ros2_control/Worx code paths against the firmware emulator, with Nav2,
coverage and mowing logic, and no Webots:

```bash
OM_MAP_PATH=map.geojson OM_DATUM_LAT=<lat> OM_DATUM_LONG=<lon> \
  ros2 launch open_mower_next worx_sim.launch.py            # python launch, with rviz (config/worx_sim.rviz)

OM_MAP_PATH=map.geojson OM_DATUM_LAT=<lat> OM_DATUM_LONG=<lon> \
  ros2 run open_mower_next om_launch \
    $(ros2 pkg prefix open_mower_next)/share/open_mower_next/config/launch/worx_sim.yaml   # as on the robot

ros2 service call /mower_logic/start_mowing std_srvs/srv/Trigger
```

The upstream Webots simulation (`make sim`) is unchanged, see [docs/simulator.md](docs/simulator.md).

## Documentation

Worx fork:

- [Worx hardware](docs/architecture/worx-hardware.md) – SPI protocol, firmware fields, emergencies
- [om_launch](docs/architecture/om-launch.md) – launcher, component container, manifests, memory
- [Mowing logic](docs/architecture/mower-logic.md) – mission tree, passes, bumps and felt obstacles
- [Coverage planner](docs/architecture/coverage-planner.md)
- [Web UI](docs/architecture/web-ui.md)

Upstream OpenMowerNext (generic, partly describes the OpenMower hardware and devcontainer setup):

- [Getting started](docs/getting-started.md), [Configuration](docs/configuration.md),
  [Dev container](docs/devcontainer.md), [Simulator](docs/simulator.md), [Visualisation](docs/visualisation.md)
- Architecture: [ROS workspace](docs/architecture/ros-workspace.md), [Localization](docs/architecture/localization.md),
  [Map server](docs/architecture/map-server.md), [Map recorder](docs/architecture/map-recorder.md),
  [Docking helper](docs/architecture/docking-helper.md), [Mainboard firmware](docs/architecture/omros2-firmware.md)
- Published upstream at [jkaflik.github.io/OpenMowerNext](https://jkaflik.github.io/OpenMowerNext/)

Build the docs locally with `cd docs && npm ci && npm run docs:dev`.

## Credits and license

Apache 2.0, see [LICENSE](LICENSE).

- OpenMowerNext by Kuba Kaflik and contributors; OpenMower by Clemens Elflein.
- `ftc_controller` derives from Clemens Elflein's FTC local planner (BSD license, notice in the source).
- `worx_hardware`, `lsm6dsv_imu`, `wheel_scale` and `coverage_planner` by Daniel Wiegert.
