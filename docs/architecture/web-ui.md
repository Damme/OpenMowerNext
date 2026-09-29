---
title: Web UI
---
# {{ $frontmatter.title }}

## Overview

`open_mower_next::web_ui::WebUiNode` is a small control page for the robot, served by the robot's own `core`
process (Worx manifest `config/launch/worx.yaml`). There's no nginx, MQTT broker or rosbridge: one
[Boost.Beast](https://www.boost.org/libs/beast) thread serves the page (compiled into the library) and a WebSocket.

The page shows the map (areas, dock, robot, trail), battery, GPS, motor and mower_logic state. It has:

- **Mow**: start mowing, go home, stop, skip pass/area, reset mission, clear emergency, motors on/off.
- **Drive**: a joystick (touch or mouse) and arrow keys / WASD, and the blade on/off switch.
- **Blade**: PWM and direction (sign) the blade runs at when it's switched on by hand.
- **Areas**: switch mowing areas on/off, remove areas, edit an area's outline (see [Map editor](#map-editor)).
  Tap an area on the map to find it in the list.
- **Bumps**: what the mower learned by bumping: obstacles it felt its way around (drawn purple on the map, the
  bumper outline at each touch; kept until Reset mission or a new mission from the beginning) and edge corrections (dashed circles, where the outline runs further in
  after perimeter bumps). Forget one (tap it on the map, or the list) or all of them. Many corrections along one
  edge mean that boundary is worth re-recording.
- **Record**: record an area boundary (automatic points while driving, or points by hand) and the docking station.
- **Log**: results of commands, including those sent from other open pages.

Missions switch the blade themselves. The page only switches it by hand (see below).

## Access

Only the VPN reaches it: the server binds `bind_address` (default `10.99.99.99`, the robot's WireGuard address,
bound even before the interface is up) and drops every connection from outside `allow` (default `10.99.99.0/24` and `10.42.40.0/22`, the VPN and
the home network routed through it).
A WebSocket must come from the page itself (`Origin` = `Host`), so another web page open in the same browser can't
drive the robot. There's no login: whoever reaches the VPN address can use it.

| Environment | Default | |
|---|---|---|
| `OM_WEB_BIND` | `10.99.99.99` | listen address |
| `OM_WEB_PORT` | `8090` | port (ports below 1024 need privileges the rootless stack doesn't have) |
| `OM_WEB_ALLOW` | `10.99.99.0/24,10.42.40.0/22` | allowed client networks, comma separated |

## How it drives the robot

Everything goes through interfaces that already exist:

| Page | ROS |
|---|---|
| mission buttons | `/mower_logic/{start_mowing,go_home,stop,skip_pass,skip_area,reset_mission,clear_emergency}` (Trigger) |
| motors on/off | `/worx/motors_enabled` (SetBool) |
| blade on/off | `/worx/manual_mow` (SetBool) |
| blade PWM / direction | parameter `manual_mow_pwm` of `/worx_hardware` (`/worx_hardware/set_parameters`) |
| joystick | `/cmd_vel_joy` (TwistStamped) → twist_mux, priority above navigation |
| areas on/off | rewrites mower_logic's `disabled_areas_file`, read when a new mission is planned |
| remove area | `/remove_area` (map_server) |
| save an edited area | `/save_area` (map_server, same id = update) |
| recording | `/record_area_boundary`, `/record_docking_station` (actions), `/set_recording_mode`, `/add_boundary_point`, `/finish_area_recording` (map_recorder) |

## Motors

mower_logic switches the motors itself (`auto_motors`): on as soon as it leaves a parked state (`IDLE`,
`CHARGING`, `WAITING_FOR_RAIN`, `EMERGENCY`), off once it has been parked for `motors_off_delay` (2 s). It only
switches when that changes, so the Motors on/off buttons still work in between (off → on is also how the Worx
firmware resets an emergency).

Manual driving switches the motors on by itself, and they go off again after `motors_idle_timeout` (25 s) without
joystick input or a running manual blade, when the last page closes, or never if a Motors button was pressed since
(then they stay as set). The manual blade needs the motors on already.

Manual driving is accepted only while mower_logic's command is `IDLE`. Speeds are scaled on the robot to
`max_linear` / `max_angular` (0.3 m/s, 1.0 rad/s), and a zero command follows `joy_timeout` (0.3 s) after the last
message, or at once when the page closes or loses focus. twist_mux's own 0.5 s timeout stays behind it.

## Blade by hand

The blade can only be switched on while mower_logic's command is `IDLE` and the motors are on. It runs at
`manual_mow_pwm` (default 1850, within ±`manual_mow_max_pwm`; negative = reverse). It goes off:

- when the page that switched it on closes,
- when Start mowing / Go home is pressed, or mower_logic leaves `IDLE` in any other way (while a page is open),
- in worx_hardware: emergency, lift, bump, motors off, board link loss, and `blade_idle_timeout` (25 s) without
  drive commands. It then stays off until it's switched on again.

The PWM can change while the blade runs; the direction only while it's off. Blade on/off/PWM results appear in the log.

## Map editor

Small outline fixes without exporting the map (Areas tab → Edit, any area type), made for a computer with a mouse.
The page loads the area at full resolution (the map it draws is simplified to 2 cm) and edits a copy; nothing
reaches the robot before Save.

- Drag a point, or a line (both ends move). Click a line to add a point. Click a point to select it; Range (or
  Shift-click) selects the stretch up to the next point clicked (the shorter way round), Ctrl-click adds a point. A
  selection moves together: drag, or arrow keys (1 cm, Shift 10 cm).
- Simplify: Douglas-Peucker like JOSM's Simplify way: drops every point less than the tolerance (default 4 cm) off
  the line through the points kept around it. On the selected stretch only (its ends stay), or the whole outline.
- Delete points (at least 3 stay), Undo / Redo (Ctrl+Z / Ctrl+Y, 100 steps), rename, Delete area.
- Save warns when the outline crosses itself. Saving and removing only work while mower_logic's command is `IDLE`.

map_server stores and publishes the new outline like any other change. The area's recorded stance (where the body
stood while recording, `grid.stance_*` in map_server) is kept only within 5 cm of the new outline: along a moved line
the body never stood there, so the grid there comes from the polygon alone. A saved mission notices the change
(plan fingerprint) and replans that area.

## Resources

Subscriptions and service/action clients exist only while a page is open. With no page open, the component is a
thread waiting on its socket plus one publisher, and it receives no status traffic. The library is built with
`-Os` and hidden symbols (exporting Beast's template names alone cost ~1.9 MB). Measured in the sim's `core`
process (x86, PSS): ~1.5 MB of library pages and ~0.5 MB heap with no page open, ~0.1 MB more with a page open;
two threads (server, executor).

## Protocol

JSON text messages over `ws://<host>:<port>/ws`.

- Robot → page: `{"t":"state"}` (2 Hz), `{"t":"pose","x","y","yaw"}` (5 Hz, from `pose_topic`,
  default `/odometry/filtered/map`), `{"t":"map","areas":[…],"docks":[…]}` (on connect and when the map changes;
  polygons simplified to 2 cm), `{"t":"rec","pts":[…]}` (boundary being recorded), `{"t":"log","ok","msg","time"}`,
  `{"t":"obs","depth","obstacles":[{"id","lines":[[[x,y],…],…]}],"edges":[{"x","y","offset","radius"}]}` (mower_logic's
  `~/obstacles`, on connect and when it changes).
- Page → robot: `{"c":"logic","name":…}`, `{"c":"motors","on":…}`, `{"c":"joy","v":-1..1,"w":-1..1}`, `{"c":"blade","on":…}`, `{"c":"blade_pwm","pwm":…}`,
  `{"c":"area","id":…,"enabled":…}`, `{"c":"area_remove","id":…}`, `{"c":"area_get","id"}` (answered to that page
  only with `{"t":"area","id","name","type","pts"}`, mm), `{"c":"area_save","id","name","pts":[[x,y],…]}`
  (3–10000 points; an empty name keeps the old one), `{"c":"rec_start","name","type","auto"}`,
  `{"c":"rec_auto","auto"}`, `{"c":"rec_point"}`, `{"c":"rec_finish"}`, `{"c":"rec_cancel"}`,
  `{"c":"dock_start","name"}`, `{"c":"dock_cancel"}`, `{"c":"forget","kind","x","y"}` (mower_logic's
  `~/forget_obstacle`: `obstacle` / `edge` nearest to x, y within 0.6 m, `all_obstacles`, `all_edges`).

Source: `src/web_ui/` (`web_server.*` ROS-free server, `web_ui_node.cpp`, `index.html`).
