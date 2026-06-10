# Lidar Power Management — Design Document

## Current Setup

The lidar driver is launched directly inside `diffbot.launch.py` as a plain `Node`:

- **Model**: LDLiDAR LD06
- **Port**: `/dev/ttyAMA4` (no conflict with the Arduino, which uses `/dev/ttyUSB0`)
- **Baudrate**: 230400
- **Frame ID**: `lidar` (TF is provided by robot_state_publisher via the URDF — no
  separate static transform publisher is needed)

```python
# Current entry in diffbot.launch.py
lidar_node = Node(
    package='ldlidar_stl_ros2',
    executable='ldlidar_stl_ros2_node',
    name='LD06',
    parameters=[
        {'product_name': 'LDLiDAR_LD06'},
        {'topic_name': 'scan'},
        {'frame_id': 'lidar'},
        {'port_name': '/dev/ttyAMA4'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False}
    ]
)
```

---

## Problem

The LDLidar driver (`ldlidar_stl_ros2_node`) is a C++ binary that connects to the
lidar over serial at startup. If the lidar is not powered on, the driver calls
`exit(EXIT_FAILURE)` within 3 seconds and the process dies. ROS2 launch has no built-in
mechanism to restart a node only when a hardware condition (power) becomes true.

This causes three failure modes:

| Scenario | Result |
|---|---|
| Bringup launched, lidar power off | Driver starts, fails immediately, stays dead |
| Lidar powered on after bringup | Driver is not running, nothing restarts it |
| Lidar powered on first, then driver started | Works, but initial serial errors flood the log |

The root cause is in `ldlidar_stl_ros2/src/demo.cpp` lines 95–107: the driver calls
`exit(EXIT_FAILURE)` on both serial open failure and `WaitLidarCommConnect` timeout
with no retry logic.

---

## Proposed Solution

Write a new **lidar manager node** (`lidar_manager.py`) that acts as a supervisor.
It subscribes to `lidarPWR_status` (the Bool feedback from `arduino_bridge.py`) and
manages the lidar driver process lifetime:

- When `lidarPWR_status` goes **True**: wait 2 seconds for the serial port `/dev/ttyAMA4`
  to become ready, then spawn the lidar driver as a subprocess.
- When `lidarPWR_status` goes **False**: terminate the driver subprocess gracefully.
- On bringup startup: wait for the first `lidarPWR_status` message — do not assume
  a default state.

The lidar driver itself is **not modified**. The TF transform (`base_link → lidar`)
continues to be provided by robot_state_publisher via the URDF and requires no changes.

---

## Why Not Other Approaches

**`respawn=True` in launch**: The driver exits in ~3 s when unpowered. Respawn would
just spam the logs with restart attempts at full rate with no power-awareness.

**`RegisterEventHandler(OnProcessExit)` in launch**: Could re-launch the node but
still has no condition tied to `lidarPWR_status`. Same spam problem.

**Modify the C++ driver for reconnect**: Works but means maintaining a fork of a
third-party package. Complicates future upstream updates.

**Run lidar manager as a subprocess of another node**: Introduces unnecessary coupling.
A dedicated node is cleaner and independently testable.

---

## Files to Create or Modify

### 1. NEW — `scripts/lidar_manager.py`

New Python ROS2 node. Responsibilities:

- Subscribe to `lidarPWR_status` (Bool)
- Track previous state for edge detection (only act on changes)
- **Rising edge (False → True)**:
  - Log "lidar power on, waiting for port to become ready…"
  - `time.sleep(2.0)` — tunable via ROS2 parameter `lidar_startup_delay_s`
  - Spawn the driver directly via `subprocess.Popen` with `ros2 run` and all
    required parameters (avoids a nested launch process):
    ```
    ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node \
      --ros-args \
      -p product_name:=LDLiDAR_LD06 \
      -p topic_name:=scan \
      -p frame_id:=lidar \
      -p port_name:=/dev/ttyAMA4 \
      -p port_baudrate:=230400 \
      -p laser_scan_dir:=true \
      -p enable_angle_crop_func:=false
    ```
  - Store the `Popen` handle
- **Falling edge (True → False)**:
  - Log "lidar power off, stopping driver…"
  - Send `SIGTERM` to the subprocess; if it does not exit within 3 s, send `SIGKILL`
  - Clear the handle
- Publish `lidar_active` (Bool) so other nodes (e.g. Nav2) can check if scan data
  is valid before using it
- On node shutdown: terminate any running subprocess

**Edge cases to handle:**
- `lidarPWR_status=True` arrives before the previous subprocess has fully exited →
  kill it first, then start fresh
- `lidarPWR_status=True` arrives at startup when lidar was already on (e.g. manual
  test) → start driver immediately (no previous subprocess to worry about)
- Port `/dev/ttyAMA4` not accessible → log a clear error, publish `lidar_active=False`,
  do not crash

### 2. MODIFY — `bringup/launch/diffbot.launch.py`

- **Remove** the `lidar_node` `Node(...)` definition and its entry from the `nodes`
  list — the manager takes over its lifecycle.
- **Add** the `lidar_manager` node to the `nodes` list so it starts with the rest of
  the robot hardware.

No new launch file is needed. The manager lives alongside the other hardware nodes
in `diffbot.launch.py`, which is the right place since the lidar is robot hardware.

### 3. MODIFY — `CMakeLists.txt` (hoverboard_driver package)

Add `lidar_manager.py` to the installed scripts list so `ros2 run hoverboard_driver
lidar_manager.py` works and the node is on `$PATH` after `colcon build`.

---

## Startup Sequence (Happy Path)

```
bringup.launch.py
  ├── arduino.launch.py   → arduino_bridge starts, sends lidarPWR=0 to Arduino
  ├── diffbot.launch.py
  │     ├── control_node, robot_state_pub_node, joint_state_broadcaster, …
  │     └── lidar_manager node ← starts, waits for lidarPWR_status
  │         (lidar_node entry removed from here)
  ├── gps.launch.py
  ├── camera.launch.py
  └── navigation.launch.py

  ~100 ms later:
  arduino_bridge receives Arduino feedback → publishes lidarPWR_status=False
  lidar_manager receives False → does nothing

  User presses joystick button 3:
  joy_to_arduino publishes lidarPWR_cmd=True
  arduino_bridge sets pin high → Arduino confirms → publishes lidarPWR_status=True
  lidar_manager receives True (rising edge)
    → sleeps 2 s (lidar_startup_delay_s)
    → subprocess.Popen("ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args …")
    → publishes lidar_active=True

  User presses button 3 again:
  lidarPWR_status=False
  lidar_manager sends SIGTERM to subprocess (SIGKILL after 3 s if needed)
  → publishes lidar_active=False
```

---

## Open Questions Before Implementation

1. **Nav2 and missing scan data**: When lidar is off, `nav2_collision_monitor` and
   the costmap will not receive `/scan` data. Confirm whether Nav2 should be paused,
   or whether it should just tolerate missing scan data (e.g. by setting a generous
   `expected_update_rate` timeout in `nav2_params.yaml`).

2. **Power-on delay**: 2 seconds is a conservative guess for `/dev/ttyAMA4` to be
   ready after the lidar powers on. Since `ttyAMA4` is a fixed hardware UART (not
   USB), the port itself is always present — the delay is only needed for the lidar's
   own motor/firmware to start responding. This may be tuneable down to ~0.5–1 s.
   Exposed as ROS2 parameter `lidar_startup_delay_s` so it can be adjusted at
   runtime without rebuilding.
