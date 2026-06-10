# Plan: Add Power Topic to LDLidar Driver

## Goal

Modify `ldlidar_stl_ros2/src/demo.cpp` to subscribe to a ROS2 Bool topic that
controls when the driver starts and stops the lidar. When the topic goes `true` the
driver opens the serial port and begins publishing scan data. When it goes `false` the
driver closes the serial port and idles — but the node stays alive. No external manager
node is needed.

The change is **backward compatible**: if the new parameter `power_topic_name` is left
empty (the default), the driver behaves exactly as it does today.

---

## Files to Modify

| File | Change |
|---|---|
| `ldlidar_stl_ros2/src/demo.cpp` | Add parameter, subscription, and state-machine loop |
| `ldlidar_stl_ros2/CMakeLists.txt` | Add `std_msgs` dependency |
| `ldlidar_stl_ros2/package.xml` | Add `<depend>std_msgs</depend>` |
| `ros2-driver-converted/bringup/launch/diffbot.launch.py` | Pass `power_topic_name` to the lidar node |

---

## 1. `src/demo.cpp`

### 1a. Add include

At the top, alongside the existing includes:

```cpp
#include <std_msgs/msg/bool.hpp>
```

### 1b. Declare and read the new parameter

After the existing `declare_parameter` / `get_parameter` block (around line 54):

```cpp
std::string power_topic_name;
node->declare_parameter<std::string>("power_topic_name", "");
node->get_parameter("power_topic_name", power_topic_name);
```

### 1c. Add state variables before the main logic

```cpp
bool lidar_powered  = false;  // set by subscription callback
bool lidar_running  = false;  // true after Start() + WaitLidarCommConnect() succeed
bool use_power_topic = !power_topic_name.empty();
```

### 1d. Conditional startup and subscription

Replace the current unconditional `Start()` / `WaitLidarCommConnect()` block
(lines 95–107) with:

```cpp
// Create the scan publisher unconditionally (topic exists even while lidar is off)
rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher =
    node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);

std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> power_sub;

if (use_power_topic) {
  // Subscribe; callback just records the latest power state
  power_sub = node->create_subscription<std_msgs::msg::Bool>(
      power_topic_name, 10,
      [&lidar_powered](const std_msgs::msg::Bool::SharedPtr msg) {
        lidar_powered = msg->data;
      });
  RCLCPP_INFO(node->get_logger(),
      "Waiting for power signal on topic: %s", power_topic_name.c_str());

} else {
  // Original behaviour: start immediately, exit on failure
  if (!ldlidarnode->Start(type_name, port_name, serial_port_baudrate,
                          ldlidar::COMM_SERIAL_MODE)) {
    RCLCPP_ERROR(node->get_logger(), "ldlidar node start is fail");
    exit(EXIT_FAILURE);
  }
  if (!ldlidarnode->WaitLidarCommConnect(3000)) {
    RCLCPP_ERROR(node->get_logger(), "ldlidar communication is abnormal.");
    exit(EXIT_FAILURE);
  }
  lidar_running = true;
  lidar_powered = true;
  RCLCPP_INFO(node->get_logger(), "Publish topic message: ldlidar scan data.");
}
```

### 1e. Rewrite the main loop

Replace the current `while (rclcpp::ok())` loop with:

```cpp
rclcpp::WallRate r(10);
ldlidar::Points2D laser_scan_points;
double lidar_scan_freq;

while (rclcpp::ok()) {
  // Process subscription callbacks (power topic and any others)
  rclcpp::spin_some(node);

  if (use_power_topic) {
    if (lidar_powered && !lidar_running) {
      // Power just came on — try to open the serial port and connect
      RCLCPP_INFO(node->get_logger(), "Power on received, starting lidar...");
      if (ldlidarnode->Start(type_name, port_name, serial_port_baudrate,
                             ldlidar::COMM_SERIAL_MODE)) {
        if (ldlidarnode->WaitLidarCommConnect(3000)) {
          RCLCPP_INFO(node->get_logger(), "Lidar started successfully.");
          lidar_running = true;
        } else {
          RCLCPP_ERROR(node->get_logger(),
              "Lidar comm connect timeout. Will retry on next power-on signal.");
          ldlidarnode->Stop();
        }
      } else {
        RCLCPP_ERROR(node->get_logger(),
            "Lidar Start() failed. Will retry on next power-on signal.");
      }

    } else if (!lidar_powered && lidar_running) {
      // Power just went off — close the port cleanly
      ldlidarnode->Stop();
      lidar_running = false;
      RCLCPP_INFO(node->get_logger(), "Lidar stopped (power off).");
    }
  }

  // Only poll scan data when the driver is running
  if (lidar_running) {
    switch (ldlidarnode->GetLaserScanData(laser_scan_points, 1500)) {
      case ldlidar::LidarStatus::NORMAL:
        ldlidarnode->GetLidarScanFreq(lidar_scan_freq);
        ToLaserscanMessagePublish(laser_scan_points, lidar_scan_freq,
                                  setting, node, publisher);
        break;
      case ldlidar::LidarStatus::DATA_TIME_OUT:
        RCLCPP_ERROR(node->get_logger(),
            "get ldlidar data is time out, please check your lidar device.");
        break;
      case ldlidar::LidarStatus::DATA_WAIT:
        break;
      default:
        break;
    }
  }

  r.sleep();
}
```

**Why `spin_some` instead of `spin`:** `rclcpp::spin` blocks forever and would prevent
the loop from running. `spin_some` processes any queued callbacks and returns immediately,
so the scan-polling loop keeps ticking at 10 Hz while still receiving power topic
updates.

**Why `Start()` / `Stop()` are safe to call repeatedly:** `Stop()` sets
`is_start_flag_ = false` and closes the serial port. The next call to `Start()` sees
`is_start_flag_ = false`, calls `comm_pkg_->ClearDataProcessStatus()` to reset
internal state, then reopens the port. This is the correct restart sequence according
to the driver's own API.

---

## 2. `CMakeLists.txt`

Add `std_msgs` in two places:

```cmake
# find dependencies — add std_msgs
find_package(std_msgs REQUIRED)

# ament_target_dependencies — add std_msgs
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs std_msgs)
```

---

## 3. `package.xml`

Add alongside the existing `<depend>` entries:

```xml
<depend>std_msgs</depend>
```

---

## 4. `diffbot.launch.py`

Add `power_topic_name` to the lidar node's parameter list:

```python
lidar_node = Node(
    package='ldlidar_stl_ros2',
    executable='ldlidar_stl_ros2_node',
    name='LD06',
    output='screen',
    parameters=[
        {'product_name': 'LDLiDAR_LD06'},
        {'topic_name': 'scan'},
        {'frame_id': 'lidar'},
        {'port_name': '/dev/ttyAMA4'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'power_topic_name': 'lidarPWR_status'},  # <-- new line
    ]
)
```

The `lidarPWR_status` topic is published by `arduino_bridge.py` as `std_msgs/Bool`.

---

## Startup Sequence After Changes

```
bringup.launch.py
  └── diffbot.launch.py
        └── ldlidar_stl_ros2_node starts
              Declares power_topic_name = "lidarPWR_status"
              Subscribes to /lidarPWR_status
              Enters idle loop (lidar_powered=false, lidar_running=false)

  arduino_bridge starts
  Sends lidarPWR=0 to Arduino → Arduino confirms → publishes lidarPWR_status=False
  lidar driver receives False → stays idle

  User presses joystick button 3:
  joy_to_arduino publishes lidarPWR_cmd=True
  arduino_bridge sets pin high → publishes lidarPWR_status=True
  lidar driver receives True
    → Start() opens /dev/ttyAMA4
    → WaitLidarCommConnect(3000) waits up to 3 s for lidar data
    → lidar_running = true
    → begins publishing /scan

  User presses button 3 again:
  lidarPWR_status=False
  lidar driver calls Stop() → closes port
  lidar_running = false → /scan stops publishing
  Node stays alive, ready for next power-on
```

---

## Build After Changes

```bash
cd ~/pi_ws
colcon build --symlink-install --packages-select ldlidar_stl_ros2
source install/setup.bash
```

---

## Notes

- The `lidar_powered` flag is written from a `spin_some` callback and read from the
  same thread in the main loop, so no mutex is needed.
- If the power topic message arrives before `arduino_bridge` is fully connected to
  the Arduino, the driver will simply wait in idle until the first `True` arrives.
- `WaitLidarCommConnect(3000)` blocks the loop for up to 3 seconds on each power-on
  event. During those 3 seconds, `spin_some` is not called. This is acceptable
  because it only happens once per power cycle and no other time-critical topic needs
  processing in that window.
- If Start() or WaitLidarCommConnect() fail (e.g. lidar not fully warmed up yet),
  the driver logs the error and goes back to idle. The next `True` message from the
  power topic will trigger another attempt. This replaces the old `exit(EXIT_FAILURE)`
  with a graceful retry.
