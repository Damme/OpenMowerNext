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
  `transit_jitter_min_distance` (4 m) across open lawn (the straight line keeps `transit_via_margin`, 1.2 m, from
  every edge) a random via point up to `transit_jitter` (0.35 m) to the side. Repeated trips then don't wear one
  track into the lawn; in corridors the transit goes direct (a via point there made an S-bend).
- Recorded boundaries are where the GPS antenna (about the robot centre) drove; the body overhung them by half the
  mower's width, so that rim is known to be clear. On Worx the planner (Smac lattice, real footprint) keeps the
  centre inside the line and the footprint inside line + rim (map_server `grid.edge_band`, `RimCostLayer`),
  nothing beyond. The inflation around exclusions and area edges keeps transits well inside where there is room.
- A bump (worx_hardware) stops the transit or pass. The bumper only says *that* the front touched something, not
  where. Each bump records a **contact**: a thin band (`bump_mark_depth` 0.10 m, `bump_mark_gap` 0.02 m outside the
  bumper) along the part of the front outline that can have touched: the whole front when driving straight, only
  the leading corner while arcing around an obstacle. Contacts whose marks come within `bump_merge_distance`
  (0.3 m) form one **felt obstacle**. Wherever the body drives forward later, the marks under it are erased (only
  driving forward: turning in place can sweep past a pin without a bump), so what is left maps the real object.
- **Trying again** (`TryAgain`): a first bump isn't believed (false detections are common). The mower backs off
  `feel_retry_backoff` (0.4 m) and mows on along the same line, blade on. Driving 15 cm past where it bumped makes it
  a false detection (state `gone`). A second bump there **confirms** it: the pass is shifted sideways around the spot
  by the next of `feel_sidesteps` (0.15, -0.15, 0.30, -0.30 m, left positive; full for `feel_sidestep_length` 0.35 m
  on each side, fading over `feel_sidestep_ramp` 0.7 m; after backing off `feel_sidestep_backoff` 1.0 m so the
  controller settles on the new line) and mowed on - a 4 cm pin costs a few cm of lawn. Only confirmed obstacles go
  to the planner's costmap (`~/bump_obstacles` → `costmap_layers::BumpLayer`, every message replaces the last; never
  under the robot's own footprint, never outside the areas).
- **Feeling around** (`FeelAround`, like a robot vacuum around a chair) when no sidestep gets past: back off
  `feel_backoff` (0.15 m), turn away `feel_turn` (40°) to the side with more room, then arc back towards the obstacle
  (radius `feel_arc_radius` 0.6 m, `feel_speed` 0.15 m/s: bumps are only detected driving forward). Each bump on the
  arc adds a contact and starts that over from there. Once it crosses the pass beyond the obstacle, the pass continues
  at the first pose after which the next `bump_lookahead` is clear of the marks. Blade per `feel_blade` (robot: on,
  `OM_FEEL_BLADE`). It gives up at `feel_max_contacts` (12), `feel_max_travel` (8 m), `feel_timeout` (150 s),
  `feel_max_offset` (2 m from the pass), after a full circle, or when the next step would leave line + rim: then it
  backs up and goes around what was felt with a transit (first pose `bump_clearance`, 0.35 m, from the marks).
- Later passes coming within `bump_avoid_radius` (0.25 m) of a confirmed obstacle's marks (up to `bump_lookahead`
  ahead) **swerve**: the pass is shifted just far enough (5 cm steps up to `bump_max_swerve`, 0.6 m) and mowed on.
  If nothing fits: a transit around it. A pass or continuation starting at a confirmed obstacle starts past it.
- A new obstacle counts once towards `max_bumps_per_pass` (4, then the rest of the pass is skipped); retries,
  sidesteps and feeling around it don't count again.
- Felt obstacles are kept (to look at them and fix the map) until `reset_mission` or a new mission from the
  beginning; a resumed mission keeps them, also across restarts (`obstacles_file`, robot: `/data/felt_obstacles.txt`,
  `obstacle x y yaw side state` per contact; the erasing isn't saved, the bands are whole again after a restart).
  `~/obstacles` (JSON, latched) has them and the edge corrections for the web UI, `~/forget_obstacle`
  (`ForgetObstacle.srv`) drops one or all of either.
- A confirmed bump on an outline pass with the robot centre within `edge_bump_distance` (0.35 m) of a recorded line is the
  edge itself (overgrown plants, GPS a few cm off), not an obstacle: the outline is shifted inward around the spot
  (`edge_correction_step` 0.10 m per bump, at most `edge_correction_max` 0.20 m, full within
  `edge_correction_radius`, fading out over `edge_correction_ramp`), and the mower backs up and keeps following the
  edge. Corrections are saved in `edge_corrections_file` (robot: `edge_corrections.txt` next to the map;
  `x y offset` per line) and applied to every new plan, so later loops and missions don't bump there again. A bump
  beyond the maximum is handled as an obstacle.
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
| `/mower_logic/reset_mission` | forget progress and the felt obstacles |
| `/mower_logic/clear_emergency` | clear a latched emergency (`/worx/emergency`, lift); refused while the robot is lifted, the bumper is pressed, or the firmware reports its own (tilt) emergency, which ROS can't reset over SPI |

`/mower_logic/forget_obstacle` (`open_mower_next/ForgetObstacle`): `kind` `obstacle` / `edge` (the one nearest to
`x`, `y` within `radius`, default 0.5 m) or `all_obstacles` / `all_edges`. A forgotten edge correction stays in the
plan being mowed; the next plan of the area doesn't have it.

`/mower_logic/state` (`std_msgs/String`, JSON, 1 Hz): state, command, mission progress, battery, docked, GPS, emergency.
`/mower_logic/obstacles` (`std_msgs/String`, JSON, latched, on change): felt obstacles (the middle line of each
contact's band) and edge corrections.
