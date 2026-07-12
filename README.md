# ros2-driver-converted (`hoverboard_driver`)

Base driver and bringup package for the Mowbot autonomous lawn mower
(ROS 2 Jazzy, rmw_zenoh, Raspberry Pi 5). Started as the
[hoverboard-driver](https://github.com/hoverboard-robotics/hoverboard-driver)
port of the ros2_control DiffBot example (hence the package name) and grew
into the robot's hardware layer: drive base, Arduino I/O bridge, and the full
sensor + Nav2 bringup.

Companion packages: [mowing_navigation](https://github.com/lepptu/mowing_navigation)
(route server + BT mission executor), [mowing_msgs](https://github.com/lepptu/mowing_msgs),
[mowbot_mqtt_bridge](https://github.com/lepptu/mowbot_mqtt_bridge),
[mowbot_robot_arduino](https://github.com/lepptu/mowbot_robot_arduino)
(firmware for the I/O board), and [mowbot_web_ui](https://github.com/lepptu/mowbot_web_ui).

## What's in here

| Path | Contents |
|---|---|
| `hardware/` | ros2_control `SystemInterface` for the hoverboard mainboard ([hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC)-style UART protocol): wheel commands/feedback for `diff_drive_controller`, plus a helper node publishing PCB telemetry (voltage, temperature, currents, connection state) |
| `scripts/arduino_bridge.py` | Serial bridge to the Arduino I/O board (see below) |
| `scripts/joy_to_arduino.py` | Gamepad buttons → Arduino commands (blade enable/RPM, hoverboard power button, spare relay, LiDAR power) with 2 Hz republish so the blade keep-alive watchdog stays fed |
| `bringup/launch/` | `bringup.launch.py` = arduino + diffbot (ros2_control, controllers, LiDAR) + gps + camera + navigation (Nav2); individual launch files usable standalone |
| `bringup/config/` | `nav2_params.yaml`, `ekf.yaml` (robot_localization), `gps.yaml`, `twist_mux.yaml` (joystick / web drive pad / Nav2), `hoverboard_controllers.yaml`, Arduino/BNO085/joystick params, slam_toolbox mapping params |
| `bringup/behavior_trees/` | `transit_to_segment.xml` (tame NavigateToPose BT for mowing transits — 2 retries, no Spin) and the default `navigate_to_pose.xml` |
| `custom_bt/` | `custom_follow_path.xml` FollowPath BT |
| `description/` | URDF/xacro + meshes; serial port configured in `description/ros2_control/hoverboard_driver.ros2_control.xacro` |
| `docs/` | LiDAR power management notes (F29 idle power-off) |
| `plans/` | `PLAN_ARDUINO_BRIDGE_CPP.md` — planned C++ rewrite of the bridge |
| `maps/`, `scripts/OLD_POIS`, `*OLD-NOT-IN-USE*` | Legacy snapshots/scripts superseded by `mowing_data`, `mow_area_recorder` and `mowing_navigation`; kept for reference |

## Arduino bridge (`arduino_bridge.py`)

Talks to the Arduino I/O board over USB serial (10-field CSV status lines in,
5-field command lines out, 10 Hz) and fans the fields out to ROS topics:

- **Inputs → topics**: `eStop_status`, `bumperFront_status`,
  `mowMotorALM_status`, `mowMotorRPM_FB_status`, `mowMotorCur_status`, plus
  echo of commanded state (`mowMotorEN_status`, `mowMotorRPM_set_status`,
  `hoverBtnR1_status`, `varaReleR2_status`, `lidarPWR_status`).
- **Topics → outputs**: `mowMotorEN_cmd`, `mowMotorRPM_set_cmd`,
  `hoverBtnR1_cmd` (+ `hoverBtnR1_pulse` for the web UI power button),
  `varaReleR2_cmd`, `lidarPWR_cmd`.
- **Safety**:
  - *Blade keep-alive watchdog* (B8/S2) — `mowMotorEN` must be refreshed;
    stale commanders can't leave the blade running.
  - *E-stop blade gate* (F30) — hardware e-stop forces blade enable off and
    latches until the commander is seen commanding it low after release.
  - *E-stop motion lock* (F31) — e-stop also stops all motion commanders,
    including the gamepad.
- **Events** (firmware ≥ 2.1.0) — `EVT:` lines from the firmware are
  republished on `arduino_events`; `EVT:BOOT`/`EVT:VER` feed a latched
  `arduino_fw_version` topic, and a BOOT arriving mid-session is logged as an
  Arduino reset (brown-out / WDT / USB glitch). Legacy `WARNING:` lines are
  logged too.

The wire protocol and the firmware side are documented in
[mowbot_robot_arduino](https://github.com/lepptu/mowbot_robot_arduino).

## Running

Deployed as the `mowbot-launch-bringup` systemd unit (unit files live in the
mowbot_web_ui repo's `robot/` directory), normally controlled from the web
UI's Launch tab. Manually:

```bash
ros2 launch hoverboard_driver bringup.launch.py     # full stack
ros2 launch hoverboard_driver diffbot.launch.py     # base + LiDAR only
ros2 launch hoverboard_driver mapping.launch.py     # slam_toolbox mapping
```

`RMW_IMPLEMENTATION=rmw_zenoh_cpp` and a running `zenoh-router` are assumed.

Hoverboard PCB wiring: USART3 (GND, TX, RX — **no VCC**) to a USB-TTL
converter or the Pi's UART; set the port in the ros2_control xacro.

## Build

```bash
cd ~/pi_ws
colcon build --packages-select hoverboard_driver --symlink-install
```

## Notes / debts

- Package/executable names still say `hoverboard_driver`/`diffbot` — renaming
  is deliberately deferred (units, launch allowlists and docs reference them).
- The wheel PID in `hardware/pid.cpp` is present but bypassed: commands go
  straight through as speed setpoints (the mainboard firmware regulates).
- `arduino_bridge.py` is slated for a C++ rewrite
  (`plans/PLAN_ARDUINO_BRIDGE_CPP.md`).
