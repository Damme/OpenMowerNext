# Worx → OpenMowerNext migration plan

Status: 2026-09-25. Compares the running ROS1 stack (`/opt/open_mower_ros`, branch `worx_claude`,
Worx mower "MrChoppie") with OpenMowerNext (ROS 2 Jazzy) and lays out what a port needs.
No ROS 2 runs on the robot yet. The plan assumes the robot moves to a **CM4** before that happens.

## TL;DR

- **Next is infrastructure without a mission layer.** Drivers (VESC + micro-ROS firmware), localization,
  Nav2, a GeoJSON map server/recorder, docking to a recorded station and a Webots sim with integration
  tests all exist. **Nothing mows yet.** There is no equivalent of `mower_logic`: `config/mow_bt.xml` is
  marked "only ideational". There's no blade/safety/battery/rain handling and no app compatibility.
- **Worx-specific work is small.** `worx_comms` is ~670 lines of our own code (the other ~19k are vendored
  rapidjson). The firmware and its JSON-over-SPI protocol stay as they are. A ros2_control hardware
  interface replaces the node. micro-ROS/rosserial would need a firmware rewrite and an SPI transport,
  so it isn't worth doing now.
- **Coverage planner: use Daniel's `coverage_planner`, not Fields2Cover.** Done: the planner is ported,
  Fields2Cover is removed as a dependency, and the output is byte-identical to the ROS1 node (see §4).
- **The big item is the mowing behaviour** (undock → plan → drive paths with blade → pause/resume on GPS or
  rain → dock). Second is the app/remote-control bridge the robot is driven with today.
- Rough size of new code: **4–6k lines**, about half of it mission logic.

## 1. Where Next stands

`origin` (Damme/OpenMowerNext) is identical to `upstream` (jkaflik/OpenMowerNext) `main` at `ba1f923`
(2026-05-24, "coverage (#36)"), so there was nothing to pull. Active work sits in **unmerged upstream
branches** (2026-06-22). `spinoff/firmware-joystick-runtime` is the newest and contains the others:
FusionCore localization, GPSFix wiring, calibration flow, navigation readiness gate, joystick, firmware
update. An `upstream` remote was added to this clone to track them.

| Area | main | newest spinoff | Notes |
|---|---|---|---|
| Hardware layer | VESC via `ros2_control` (`vesc_hw_interface`), micro-ROS agent on `/dev/ttyAMA0` | + `omros2_firmware` msgs (`/power/status`) | Only the OpenMower mainboard v0.13. `ros2_control.xacro` hardwires VESC. Firmware "safety features" are listed as not done. |
| Drive control | `diff_drive_controller`, `mower_controller` (effort group), `twist_mux` (nav/joy) | + calibration input to twist_mux | Blade is an effort joint; nothing commands it yet. |
| Localization | `robot_localization`: 2× EKF + `navsat_transform` | **FusionCore** (jkaflik fork) with GNSS rotation-heading bootstrap | The docs say heading is unknown at start ("drive in circles"). FusionCore tries to fix that. |
| GPS / RTK | `ublox_f9p` (UBX) + `ntrip_client` | + `gps_msgs/GPSFix` path | Same u-blox family as ours. |
| Navigation | Nav2: NavFn global, Regulated Pure Pursuit local, static-layer costmaps from map_server grid | + readiness gate (starts Nav2 only once localization is valid) | No FTC-like precise path follower. |
| Map | GeoJSON `map_server` (areas: operation/navigation/exclusion, docking stations, occupancy grid, Foxglove GeoJSON) | – | Different format from our `mower_map`, so a converter is needed. |
| Map recording | `map_recorder`: area boundary (manual points or auto while driving), docking station (drives on, records when charger present) | – | Driven through actions/services, e.g. from Foxglove. |
| **Docking** | **Yes**: Nav2 `opennav_docking` + our plugin `ChargerPresenceChargingDock` (docked = `/power/charger_present`), `docking_helper` actions `dock_robot_nearest` / `dock_robot_to`, staging pose 1 m in front | – | Pure pose-based, relying on GPS accuracy (`getRefinedPose` just returns the recorded pose). Undock exists in opennav_docking but nothing calls it. Covered by a Webots integration test. |
| Coverage | `coverage_server` service with Fields2Cover | – | Not wired into any behaviour. **Now replaced with our planner, see §4.** |
| Mission logic | **None** (ideational BT XML) | – | No mow/pause/resume/dock cycle, blade control, battery/rain/emergency/GPS-loss reactions, schedule. |
| UI | Foxglove layout, rosbridge (for ROS-MCP) | + foxglove bridge in launch | No OpenMower app / xbot_monitoring compatibility (out of scope per `docs/roadmap.md`). |
| Sim / CI | Webots + smoke & docking integration tests | + GNSS/degraded-GPS tests, Buildkite | Much better than our `mower_simulation`. |
| Calibration | – | `scripts/calibrate_robot.py` (odometry/speed characterization, GNSS heading) | Useful for Worx wheel ticks and speed. |

## 2. Our ROS1 stack, mapped to Next

| ROS1 (ours) | Own LOC | Next equivalent | Port action | New code |
|---|---:|---|---|---:|
| `worx_comms`: SPI JSON link to Worx board (PWM left/right/blade, motor enable/disable, wheel ticks, battery, charger, buttons/lift/rain, emergency, board IMU) | 670 | VESC `ros2_control` + micro-ROS firmware | New `worx_hardware` ros2_control **SystemInterface** plugin plus status publishers (§3) | 800–1200 |
| `lsm6dsv_imu` (I²C on the Pi) | 520 | IMU comes from firmware via micro-ROS | Straight rclcpp port, publish `/imu/data_raw` | ~400 |
| `xbot_positioning` (own EKF, GPS-timeout fixes of 2026-09-25) | 830 | robot_localization / FusionCore | Use Next's (FusionCore). Port ours only if FusionCore can't handle Worx odometry + our IMU in sim/field tests. | 0 (or ~900) |
| `xbot_driver_gps` (UBX) + `ntrip_client` | 2500 | `ublox_f9p` + `ntrip_client` | Use Next's; check the receiver config (`str2str` currently runs outside ROS). | config |
| `mower_map` (bag/GPX map + services) | 660 | `map_server` (GeoJSON) | One-off converter `map.bag` → GeoJSON (areas, obstacles→exclusions, docking point → docking station) | ~200 (py) |
| `mower_logic` (Idle, Undocking, Mowing, Docking, AreaRecording, PerimeterDocking; safety checks) | 4950 | **none** | New mowing BT (§5 phase 4) | 1500–2500 |
| `coverage_planner` | 1170 | `coverage_server` | **Done** (§4) | – |
| `ftc_local_planner` (+ our turn-assist / oscillation fixes) | 2290 | RPP controller | Start with RPP (`use_rotate_to_heading`). Port FTC as a Nav2 controller plugin only if RPP can't track mow lanes tightly enough. | 0 (or ~1500) |
| vendored `mbf_abstract_nav` patch | – | Nav2 | Obsolete: that actionlib race doesn't exist in Nav2. | 0 |
| `xbot_monitoring` + `xbot_remote` (app, MQTT, remote drive → `/joy_vel`) | 890 | Foxglove / rosbridge | Decide the UI: port an MQTT/app bridge, or drive recording/teleop from Foxglove (§5 phase 5) | 800–1500 |
| `twist_mux` | – | `twist_mux` (nav/calibration/joy) | Config only | – |
| `mower_simulation` | 270 | Webots | Use Webots; add a Worx URDF/proto | config |
| PerimeterDocking (uses Worx perimeter wire signal) | 380 | – | Optional, late. Pose-based opennav_docking first. Wire signal could become a `getRefinedPose` source. | ~400 |

## 3. Worx hardware: keep the firmware, add a ros2_control interface

The Worx board firmware speaks JSON frames over SPI (`SOF … EOF`, `MOTORREQ_SETSPEED {left,right,mow}`,
`MOTORREQ_ENABLE/DISABLE`, `ping`; status `Battery`, `MotorPulse`, `MotorCurrent`, `Digital`, `I2C_IMU`,
`Boundary`, `motorState`, …). Changing it to micro-ROS would mean new firmware plus a custom SPI transport
for micro-ROS. That's possible, but it's a separate project with no functional gain now.

Plan: `src/worx_hardware/` (functional dir, same package, as AGENTS.md asks):

- `WorxSystem : hardware_interface::SystemInterface`, with joints `left_wheel_joint`, `right_wheel_joint`
  and `mower_joint`, so Next's `diff_drive_controller` and `mower_controller` work unchanged.
  - `read()`: wheel position/velocity from `MotorPulse` ticks (`wheel_ticks_per_m`, direction bits).
  - `write()`: velocity command → PWM. Today this is **open loop** (`MAXSPEED` scaling of m/s in
    `velReceived`). Keep that first, then add a small PI loop on tick velocity. Blade effort → `mow` PWM.
  - SPI thread and framing are copied from `worx_comms.cpp`, including the 2026-09-25 bounds and rapidjson type fixes.
- Status side (a small companion node, or GPIO state interfaces + broadcaster): `/power` (BatteryState),
  `/power/charger_present` (Bool, **needed by the docking plugin**), emergency/lift/stuck/rain,
  `motorState`. Also a service or topic for emergency reset and motor enable.
- `description/`: a Worx variant of `ros2_control.xacro` (it's VESC-only today) and
  `config/hardware/worx.yaml` (chassis, wheel distance, blade offset/width, antenna offset).
- Keep INFO-level packet logging switchable (we still hunt bugs with it).

## 4. Coverage planner: Daniel Wiegert's coverage_planner instead of Fields2Cover (done)

`coverage_planner` is Daniel Wiegert's own standalone planner (Boost.Geometry), written to replace the
slic3r-based planner that OpenMower originally used. The Worx robot runs it as a ROS1 node today.

Why ours:
- It's tuned on the real mower: perimeter loops that retrace the recorded edge, obstacle rings merged per
  level, lane-skip grass recovery, validated U-turns, edge side, blade offset, concentric fill.
- Boost.Geometry only. Fields2Cover is a heavy source build (`custom_deps`) and Next used only its
  boustrophedon + constant headland.
- The same code runs on the robot today, so one algorithm serves both stacks.

What changed (commit `3b6f7a2` on branch `worx`):

| File | Change |
|---|---|
| `src/coverage_planner/geom.hpp` | **Verbatim copy** of the planner's `geom.hpp` (plus an author line) |
| `src/coverage_planner/coverage_planner.{hpp,cpp}` | ROS-agnostic core. Stages 1–5 are extracted verbatim from our node; the `g_*` globals became `PlanConfig` fields; ROS I/O moved out. API: `plan(Request, Params, LogFn) → Result{paths, mowable}` and `headings()`. |
| `cmake/coverage_planner.cmake` | Static lib + `coverage_planner_test` |
| `src/coverage_server/*` | ROS 2 adapter. Every planner setting is a node parameter with the ROS1 name/default. `outline_count` defaults to -1, so the request's `headland_loops` is used. |
| `src/msg/CoveragePath.msg`, `src/srv/AreaCoverage.srv` | Response now also has `CoveragePath[] paths` (kind, is_outline, path), so the BT can drive the transit between passes. `path` stays as the concatenation. `swath_angle` only applies when `optimize_sweep_angle` is false (same as ROS1). |
| `test/coverage_planner_test.cpp`, `test/coverage_server_utils_test.cpp` | Core tests (ROS-free) + adapter tests (old F2C scenarios ported) |
| `CMakeLists.txt`, `package.xml`, `custom_deps.yaml` | Fields2Cover removed, Boost added |

Verification (all on DammBurkWin):
- **Equivalence:** the unmodified ROS1 `coverage_planner.cpp`, compiled natively in a harness, and the new
  core were run on 4 fields (rectangle + obstacle, L-shape with interior/boundary/outside obstacles, narrow
  strip, round field with an obstacle near the edge) × 3 parameter sets (node defaults; follow-request +
  concentric + left edge + blade offset + turn radius; serpentine k=3 without loops). All ~70k emitted
  poses (x, y, yaw) and all log lines are **byte-identical**.
- **ROS 2:** built in `ros:jazzy-ros-base` Docker with Next's own `cmake/coverage_*.cmake`. No warnings with
  `-Wall -Wextra -Wpedantic`; 8 + 6 gtests pass. End to end: publish `/mowing_map` → `area_coverage`
  returns 10 passes (perimeter, fill, obstacle ring) with the exclusion as a hole, and the same numbers as ROS1.
- Not built: the full `open_mower_next` package (needs Nav2, docking, Webots, … in the container).

To finish when committing:
- `git rm --cached src/lib/fields2cover`: its gitlink is still in the index; the directory is unused.
- `src/lib/**` is git-ignored and reserved for `vcs import` deps (`make custom-deps --force` overwrites it).
  That's why the planner lives in `src/coverage_planner/`. The earlier June copy in `src/lib/` would never
  have been committed. Its uncommitted `utils.h` rewrite is superseded by this one.

Possible follow-up: switch the ROS1 node to this core (the node would shrink to its ROS I/O). Then both
repos carry identical `geom.hpp` + `coverage_planner.{hpp,cpp}`.

## 5. Plan

Development happens in Webots/Docker on DammBurkWin (host is Ubuntu 20.04, Jazzy needs 24.04, so Docker).
The robot stays on ROS1 until the CM4 and a working sim mowing cycle exist.

0. **Base branch.** Next `main` hasn't moved since 2026-05-24; the spinoffs carry the June work. Branch `worx` off `main`, merge upstream
   spinoffs once they land (FusionCore, readiness, calibration change localization and launch heavily).
   Avoid duplicating their work. ~~Coverage planner~~ done.
1. **Worx hardware interface** (§3) + Worx URDF/xacro + `config/hardware/worx.yaml`. Test against a
   Webots Worx model first, then on the bench with wheels up (CM4). Keep the firmware protocol untouched.
2. **Sensors & localization:** port `lsm6dsv_imu`, point FusionCore at Worx odometry + IMU + u-blox, run
   `calibrate_robot.py` for wheel ticks/speed. Compare against `xbot_positioning`; port it only if FusionCore
   falls short (GPS-loss behaviour: we need a ~1.5 s reaction, see HANDOFF §8 in open_mower_ros).
3. **Map:** `scripts/worx_map_to_geojson.py` (rosbag `map.bag` → GeoJSON with datum). Load it in map_server
   and check the occupancy grid (5 cm is what we use today).
4. **Mowing behaviour (largest item).** BehaviorTree.CPP v4 (already a dependency), either as a custom Nav2
   BT navigator or our own executor node:
   - Undock (`opennav_docking` undock) → for each operation area: `area_coverage` → for each `CoveragePath`:
     ComputePathToPose to the start → blade on → FollowPath (outline passes precise) → record progress index.
   - Conditions and reactions from `mower_logic::checkSafety`: emergency/lift, GPS quality and timeout
     (pause + resume from the progress index), stale pose, battery low → dock, rain → dock, blade temperature.
   - Dock (`docking_helper/dock_robot_nearest`), charge, resume.
   - Controller: tune RPP for 20 cm lanes; decide on an FTC port after sim results.
5. **Operator interface:** today recording and teleop use the OpenMower app via xbot_monitoring/xbot_remote.
   Choose between (a) Foxglove panels (Next's way, no app) and (b) a ROS 2 bridge that speaks the app's
   MQTT/xbot protocol (keeps the app; ~1k LOC). Decide before phase 4 ends, because the BT needs start/pause/dock commands.
6. **Later / optional:** perimeter-wire docking refinement (Worx `Boundary` signal), recovery behaviours
   (Nav2 backup/spin), heatmap, upstream PRs (coverage planner, Worx hardware as a hardware variant).

Hardware note: the Pi Zero 2 W (512 MB) runs the ROS1 stack at ~250 MB RSS (MBF 71 MB). Nav2 + DDS +
ros2_control won't fit comfortably, which is why this plan targets the CM4.

## 6. Open questions

- Which MCU/firmware repo is on the Worx board, and could it take micro-ROS later? (Only matters if we ever drop JSON/SPI.)
- App vs Foxglove (phase 5).
- Should the precise path tracking we rely on (FTC pre-rotate and lane accuracy) become a Nav2 controller port or RPP tuning?
- Upstream: offer the coverage planner and Worx hardware to jkaflik/OpenMowerNext, or keep them fork-only?
