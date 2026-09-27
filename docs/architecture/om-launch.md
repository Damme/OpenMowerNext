---
title: om_launch (robot launcher)
---
# {{ $frontmatter.title }}

## Why

The Worx robot runs on a Raspberry Pi Zero 2 W with 512 MB of RAM. With the python launch files, the robot side of
the stack took ~360 MB in the kinematic simulation (0.5 CPU, mid-mission, PSS):

| Part | Memory |
|---|---|
| `ros2 launch` (python) | ~50 MB, for the whole run |
| controller `spawner`s (python) | ~25 MB each, 3 of them at start-up |
| 10 separate node processes | 8–25 MB each; rclcpp + DDS cost ~8 MB per process |
| Nav2 container | ~190 MB, most of it memory glibc kept after `free()` |

`om_launch` replaces the python launch on the robot. The same stack then takes ~105 MB. The python launch files
still work and remain the way to run the simulation with rviz.

## Pieces

- **`om_launch <manifest.yaml> [--only a,b] [--skip a,b]`**: process supervisor without ROS libraries (~0.6 MB).
  It starts the manifest's processes and prefixes their output with the process name. A crashed process is
  restarted after 2 s, and the delay doubles (up to 30 s) while it keeps dying within 10 s. On SIGINT/SIGTERM it
  stops everything in reverse order and kills what is still running after 15 s.
- **`om_container <manifest.yaml> <process>`**: hosts the components of one manifest process. It loads any registered
  rclcpp component (Nav2 servers, `robot_state_publisher`, our nodes) and gives each one its own executor thread,
  like `component_container_isolated`. `plugin: controller_manager` is built in: it runs ros2_control's control
  loop (as `ros2_control_node`) and loads and activates the listed controllers in-process, without spawners.
- **Build-time generation** (`scripts/om_generate.py`): the URDF (`generated/worx_spidev.urdf`,
  `worx_fake.urdf`) and the controller parameters with the wheel geometry from `config/hardware/worx.yaml`
  (`generated/worx_controllers*.yaml`). `OM_WORX_MOTORS_ENABLED` is still read when the stack starts.
- Our nodes are components: `open_mower_next::map_server::MapServerNode`, `...::coverage_server::CoverageServerNode`,
  `...::docking_helper::DockingHelperNode`, `...::map_recorder::MapRecorderNode`,
  `...::mower_logic::MowerLogicNode`, `...::lsm6dsv_imu::Lsm6dsvImuNode`, and robot_localization's EKF and
  navsat_transform (`...::localization_components::Ekf`, `robot_localization::NavSatTransform`).

## Manifests

`config/launch/worx.yaml` is the robot, with the same nodes as `launch/openmower.launch.py` for `OM_HARDWARE=worx`.
`config/launch/worx_sim.yaml` is the kinematic simulation.

```yaml
env:                                  # set for all processes unless already set
  MALLOC_ARENA_MAX: "2"
processes:
  - name: nav
    ros_args: [--params-file, "$(find-pkg-share open_mower_next)/config/nav2_params.yaml"]  # process-wide
    components:
      - plugin: nav2_planner::PlannerServer
        name: planner_server
        remap: {cmd_vel: cmd_vel_raw}
        params: [file.yaml, {key: value, robot_description@file: path}]
        executor: single              # or multi (threads: N)
  - name: gps
    enabled: $(env OM_GPS_ENABLED true)
    cmd: [/path/to/program, --ros-args, ...]
    respawn: true                     # respawn_delay, required, delay, env
```

- Substitutions in manifests and parameter files: `$(env NAME)`, `$(env NAME default)` (`""` for empty),
  `$(find-pkg-share pkg)`, `$(dirname)`.
- Nav2's costmaps are child nodes: they only see **process-wide** parameter files (`ros_args`).
- Controllers only see the **controller manager's** arguments. Put the controller parameter file in its `params`.

Run it:

```bash
OM_MAP_PATH=map.geojson OM_DATUM_LAT=.. OM_DATUM_LONG=.. OM_GPS_ENABLED=false \
  install/open_mower_next/lib/open_mower_next/om_launch \
  install/open_mower_next/share/open_mower_next/config/launch/worx.yaml
```

## Memory settings

Both manifests set:

- `MALLOC_ARENA_MAX=2`: glibc otherwise gives every thread its own malloc arena, and each arena keeps its freed
  memory.
- `MALLOC_MMAP_THRESHOLD_=524288`: blocks from 512 KiB up get their own mapping and go back to the system when
  freed. Without it, glibc raises the threshold after the first big free, and Smac's search graph, its heuristic
  buffers and costmap messages stay in the heap.

The robot's Nav2 overlay `config/hardware/worx_nav2_robot.yaml` uses `LeanSmacPlannerLattice`. It is the Smac lattice
planner, but it releases its search graph and obstacle heuristic after every plan. The overlay also drops unused
behaviours and does not publish the full global costmap.

Measured in the kinematic sim (mow_6, CycloneDDS, x86 limited to 0.5 CPU, PSS mid-mission):

| Setup | Nav2 | Robot side | Launcher |
|---|---|---|---|
| python launch, glibc defaults | 193 MB | 307 MB | 50 MB |
| python launch + both malloc settings | 61 MB | 158 MB | 49 MB |
| om_launch (composed) + lean planner + malloc settings | 52–55 MB | 103–105 MB | 0.6 MB |

On the Pi Zero 2 W (same kinematic sim with the software Worx board, mow_6): 143 MB PSS+swap for the whole stack
mid-mission, 283 MB of the 455 MB still available, ~90 % CPU of 400 % (with Nav2's lifecycle bonds off – see
`worx_nav2_robot.yaml`; they took ~25 % of a core in the lifecycle manager alone).
