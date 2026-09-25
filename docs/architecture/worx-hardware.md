---
title: Worx hardware
---
# {{ $frontmatter.title }}

## Overview

Worx Landroid mowers run with the Worx mainboard and its firmware, which talks JSON over SPI.
The `open_mower_next/WorxSystem` [ros2_control](https://control.ros.org) hardware plugin drives that board.
The firmware protocol is unchanged, so the same board works with open_mower_ros and OpenMowerNext.

Select it with `OM_HARDWARE=worx`. That picks `config/hardware/worx.yaml` (dimensions, plugin settings),
`description/ros2_control_worx.xacro`, the Worx Nav2 overrides (`config/hardware/worx_nav2.yaml`), and starts the
LSM6DSV IMU node. The micro-ROS agent isn't started.

## Protocol

- Full-duplex SPI (`/dev/spidev0.0`, mode 0, 1.5 MHz), fixed 250-byte transfers every 1 ms.
- A message is `0x01` + JSON + `0xFF`; unused bytes are `0x00`. Received messages may span transfers.
- Host → board: `MOTORREQ_SETSPEED {left, right, mow}` (PWM), `MOTORREQ_ENABLE`, `MOTORREQ_DISABLE`,
  `ping {count}` every 2 s (resets the firmware's SPI watchdog).
- Board → host: `MotorPulse` (cumulative tick magnitudes + direction bits, blade pulses, `Emergancy`, `BlockForward`),
  `Battery` (`mV`, `mA`, `Temp`, `InCharger`), `MotorCurrent`, `MotorPWM`, `Digital`, `Analog`, `Boundary`,
  `motorState`, `powerState`, and `DEBUG` text lines.

## Interfaces

| Joint | Command | State |
|---|---|---|
| `left_wheel_joint`, `right_wheel_joint` | velocity (rad/s) → PWM = v·`pwm_per_mps`, open loop | position, velocity from `MotorPulse` (`wheel.ticks_per_m`) |
| `mower_joint` | effort 0..1 → `mow_pwm` | velocity = blade pulses |

| Topic / service | Type | |
|---|---|---|
| `/power` | `sensor_msgs/BatteryState` | voltage, charge current, percentage from `battery_empty/full_voltage` |
| `/power/charger_present` | `std_msgs/Bool` | `InCharger`; used by the docking plugin |
| `/worx/status` | `open_mower_next/WorxStatus` | link, emergency, motor state, raw digital/analog/boundary values |
| `/worx/emergency` | `std_srvs/SetBool` | latch/clear a software emergency (all outputs 0) |
| `/worx/motors_enabled` | `std_srvs/SetBool` | `MOTORREQ_ENABLE` / `_DISABLE` |

`Digital` inputs are published raw and as "active". Inputs listed in `digital_inverted`
(default `Door,Door2,Lift,Collision`) read 1 in their normal state.

## Safety

- `/worx/emergency` zeroes all outputs until it's cleared.
- The blade stays off after an emergency, link loss or motor disable until its command has been 0 once, so a held
  command can't restart it.
- The blade stops after `blade_idle_timeout` (25 s) without drive commands.
- Without board messages for `link_timeout` all outputs are 0.

## Bench test and firmware emulator

`worx_transport:=fake` replaces SPI with an emulator of the firmware (ticks follow the commanded PWM, battery charges
while "in the charger"). It's for tests and for running without a robot:

```bash
OM_HARDWARE=worx ros2 launch open_mower_next hardware_bench.launch.py worx_transport:=fake
```

`launch/worx_sim.launch.py` builds a kinematic simulation on top of it (Nav2, map, coverage, mowing logic).
