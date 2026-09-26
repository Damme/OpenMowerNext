---
title: Mowing logic
---
# {{ $frontmatter.title }}

## Overview

`mower_logic` runs the mowing mission as a [BehaviorTree.CPP](https://www.behaviortree.dev) v4 tree
(`config/mower_logic.xml`). It's modelled on the behaviours of open_mower_ros.

Priorities, re-checked on every tick (a higher branch halts a lower one):

1. **Emergency** (`/worx/status`): hold, blade off.
2. **Charging**: below `battery_low`, dock and wait until `battery_resume`, then continue the mission.
3. **Rain** (`dock_on_rain`): dock and wait until it has been dry for `rain_clear_delay`.
4. **Mowing** (command `MOW`): paused while GPS is bad (`gps_timeout`), continues after `gps_settle` s of good fixes.
5. **Go home** (command `HOME`): dock.
6. **Idle**.

## Mowing

- Undock (Nav2 docking server), then for each operation area (all, or `areas`) plan with `area_coverage`.
- For each pass: a transit to its start, then `follow_path` with the pass controller (`controller_id`; FTC on Worx)
  and the blade on.
- The transit uses `navigate_through_poses` with `config/behavior_trees/transit_to_pass.xml`: a position-only goal
  (FTC turns to the pass heading in place), replanning only when the path becomes invalid, and on legs longer than
  `transit_jitter_min_distance` (4 m) a random via point up to `transit_jitter` (0.6 m) to the side, at least
  `transit_via_margin` (0.8 m) from area edges and exclusions. Repeated trips then don't wear one track into the lawn.
- On Worx the planner (Smac lattice) checks the real footprint, and the inflation around exclusions and the area
  edges keeps transits away from them where there is room.
- A bump (worx_hardware) stops the transit or pass: back up `bump_backup` (0.3 m), mark the obstacle
  (`~/bump_obstacles` → costmaps: a strip across the whole front, since the sensor doesn't say where it was hit;
  kept clear of the robot's own footprint including the inflation), continue the pass at the first pose
  `bump_clearance` (1.2 m) past it. More than `max_bumps_per_pass` bumps, or an obstacle at the end of a pass,
  skips the rest of the pass.
- Progress is tracked along the pass, so a pause (GPS, emergency, charging) resumes where it stopped, a little
  (`resume_backtrack`) before the stop point.
- If the robot is already within `resume_direct_distance` (0.3 m) of the pass (after a GPS or emergency pause, or when
  the next pass starts where the last one ended), it skips the transit and starts from the pose next to it.
  FTC aligns in place. The backtracked start can't be used then: it lies behind the robot and FTC is forward-only.
- A pass that fails `max_pass_attempts` times is skipped (2 s between attempts). After `max_skipped_passes_in_row`
  (3) skipped passes the mission stops and the mower goes home: something beyond one pass is wrong.

Only `FollowPass` switches the blade on (after `blade_spinup`). It switches it off on success, failure and halt,
and the executor forces the blade off whenever no pass is running.

## Commands and state

| Service (`std_srvs/Trigger`) | |
|---|---|
| `/mower_logic/start_mowing` | start or continue the mission |
| `/mower_logic/go_home` | dock, keep the mission |
| `/mower_logic/stop` | idle where it is, keep the mission |
| `/mower_logic/skip_pass`, `/mower_logic/skip_area` | skip ahead |
| `/mower_logic/reset_mission` | forget progress |
| `/mower_logic/clear_emergency` | clear a latched emergency (`/worx/emergency`, lift); refused while the robot is lifted, the bumper is pressed, or the firmware reports its own (tilt) emergency, which ROS can't reset over SPI |

`/mower_logic/state` (`std_msgs/String`, JSON, 1 Hz): state, command, mission progress, battery, docked, GPS, emergency.
