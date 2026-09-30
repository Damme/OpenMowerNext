---
title: Worx hardware
---
# {{ $frontmatter.title }}

## Overview

Worx Landroid mowers run with the Worx mainboard and its firmware, which talks JSON over SPI.
The `open_mower_next/WorxSystem` [ros2_control](https://control.ros.org) hardware plugin drives that board.
The protocol is the one open_mower_ros used. The firmware from 2026-09 (LandLord branch `worx-next`) adds fields and
one command; the plugin works with both.

Select it with `OM_HARDWARE=worx`. That picks `config/hardware/worx.yaml` (dimensions, plugin settings),
`description/ros2_control_worx.xacro`, the Worx Nav2 overrides (`config/hardware/worx_nav2.yaml`), and starts the
LSM6DSV IMU node. The micro-ROS agent isn't started.

## Protocol

- Full-duplex SPI (`/dev/spidev0.0`, mode 0, 1.5 MHz), fixed 250-byte transfers every 1 ms.
- A message is `0x01` + JSON + `0xFF`; unused bytes are `0x00`. Received messages may span transfers.
- Host → board: `MOTORREQ_SETSPEED {left, right, mow}` (PWM; a negative `mow` runs the blade in reverse),
  `MOTORREQ_ENABLE`, `MOTORREQ_DISABLE`, `MOTORREQ_RESETEMG`, and `ping {count}` every 2 s. Any valid command resets
  the firmware's SPI watchdog (older firmware: only `ping`).
- Board → host: `MotorPulse` (20 Hz: cumulative tick magnitudes + direction bits, blade pulses, `Emergancy`,
  `BlockForward`), `Battery` (`mV`, `mA`, `Temp`, `InCharger`), `MotorCurrent`, `MotorPWM`, `Digital`, `Analog`,
  `Boundary`, `motorState`, `powerState`, and `DEBUG` text lines. `Battery`, `Digital`, `Analog`, `MotorPWM` and
  `MotorCurrent` take turns, each every 1.25 s.

Added by the 2026-09 firmware:

| Field | Meaning |
|---|---|
| `MotorPulse.EmgReason` | bits of the latched emergency: 1 tilt, 2 STOP key, 4 lift (motors on, 100 ms) |
| `MotorPulse.Motors` | real motor enable state |
| `MotorPulse.Bumps` | debounced bumper presses since boot (Stuck/Stuck2/Collision low for 5 ms) |
| `MotorPulse.ms` | board time of the sample; the plugin computes wheel speed over it |
| `Battery.State`, `.Contact`, `.Enable` | `powerState`, the `CHARGER_CONNECTED` and `CHARGER_ENABLE` pins |
| `Digital` | also sent at once when an input changes |
| `motorState` | the real state (`MOTORREQ_ENABLE`/`_DISABLE`/`_IDLE`/`_EMGSTOP`), not the last request |

A firmware emergency stays latched: `MOTORREQ_ENABLE` is refused and `SETSPEED` ignored until `MOTORREQ_RESETEMG`,
which the firmware refuses while the STOP key or the lift sensor is still active.

## Interfaces

| Joint | Command | State |
|---|---|---|
| `left_wheel_joint`, `right_wheel_joint` | velocity (rad/s) → PWM = v·`pwm_per_mps`, open loop | position, velocity from `MotorPulse` (`wheel.ticks_per_m`) |
| `mower_joint` | effort 0..1 → `mow_pwm`, direction = sign of `manual_mow_pwm` | velocity = blade pulses |

| Topic / service | Type | |
|---|---|---|
| `/power` | `sensor_msgs/BatteryState` | voltage, charge current, percentage from `battery_empty/full_voltage` |
| `/power/charger_present` | `std_msgs/Bool` | `InCharger`; used by the docking plugin |
| `/worx/status` | `open_mower_next/WorxStatus` | link, emergency, motor state, raw digital/analog/boundary values |
| `/worx/emergency` | `std_srvs/SetBool` | latch/clear a software emergency (all outputs 0) |
| `/worx/motors_enabled` | `std_srvs/SetBool` | `MOTORREQ_ENABLE` / `_DISABLE` |
| `/worx/manual_mow` | `std_srvs/SetBool` | blade on by hand at `manual_mow_pwm` (overrides the `mower_joint` effort) |

The runtime parameter `manual_mow_pwm` (`ros2 param set /worx_hardware manual_mow_pwm -1500`) is the blade PWM for
`/worx/manual_mow`; the sign is the direction. It's limited to ±`manual_mow_max_pwm`, and its sign can't change while
the manual blade runs. Both start from `config/hardware/worx.yaml`. Its sign is also the direction of the commanded
(`mower_joint`) blade, taken whenever that blade starts from 0: a change made while a mission cuts applies at the
next blade start (next pass, or stop + resume).

`Digital` inputs are published raw and as "active". Inputs listed in `digital_inverted`
(default `Door,Door2,Lift,Collision`) read 1 in their normal state.

## Safety

- `/worx/emergency` zeroes all outputs until it's cleared. A firmware emergency (`Emergancy` 1) latches it too, with
  the reason in the log. Clearing it sends `MOTORREQ_RESETEMG`; if the board still reports the emergency 1.5 s
  later (cause still present), it latches again.
- A `Collision` in `Digital` or a new `Bumps` count while driving forward is a bump: forward motion blocked, blade off.
- The blade stays off after an emergency, link loss or motor disable until its command has been 0 once, so a held
  command can't restart it.
- The blade stops after `blade_idle_timeout` (25 s) without drive commands.
- The manual blade (`/worx/manual_mow`) is refused while any of these apply, and switched off (not paused) when one
  does: it has to be switched on again.
- Without board messages for `link_timeout` all outputs are 0.

## Bench test and firmware emulator

`worx_transport:=fake` replaces SPI with an emulator of the firmware (ticks follow the commanded PWM, battery charges
while "in the charger"). It's for tests and for running without a robot:

```bash
OM_HARDWARE=worx ros2 launch open_mower_next hardware_bench.launch.py worx_transport:=fake
```

`launch/worx_sim.launch.py` builds a kinematic simulation on top of it (Nav2, map, coverage, mowing logic).
