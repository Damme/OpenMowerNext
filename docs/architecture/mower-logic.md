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
- For each pass: `navigate_to_pose` to its start (transit controller), then `follow_path` with the pass controller
  (`controller_id`; FTC on Worx) and the blade on.
- Progress is tracked along the pass, so a pause (GPS, emergency, charging) resumes where it stopped, a little
  (`resume_backtrack`) before the stop point.
- A pass that fails `max_pass_attempts` times is skipped.

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

`/mower_logic/state` (`std_msgs/String`, JSON, 1 Hz): state, command, mission progress, battery, docked, GPS, emergency.
