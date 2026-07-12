# Plan: Port `arduino_bridge.py` + `joy_to_arduino.py` to C++ as a standalone ROS 2 Jazzy package

> Status: **PLANNED 2026-07-10 (rev 2) — no code written yet.**
>
> Goal: replace **both** Python nodes that talk the Arduino command
> topics — `scripts/arduino_bridge.py` (serial bridge) and
> `scripts/joy_to_arduino.py` (gamepad → command topics) — with
> functionally identical C++ nodes in a **new standalone ROS 2 Jazzy
> package + git repo** at `/home/ros-pi/pi_ws/src/mowbot_arduino_bridge`.
> All Finnish comments and log strings are **translated to English** in
> the port (topic/parameter names are public interface and stay as-is,
> Finnish-derived or not). The Arduino Nano firmware
> (`/home/ros-pi/Arduino/mowbot_robot_arduino/src/main.cpp`) is **not
> changed** — the serial protocol stays byte-for-byte identical.
>
> Launch entry points stay the same:
> - `bringup.launch.py` → `arduino.launch.py` (this repo) launches the
>   bridge — only the `Node(...)` inside `arduino.launch.py` is repointed.
> - `joystick.launch.py` (this repo) launches `joy_to_arduino` alongside
>   `joy_node` + `teleop_node` — only its `Node(...)` is repointed.
>
> Companion docs: `plans/PLAN_ESTOP_BLADE_GATE.md` (the F30 e-stop blade
> gate that MUST be ported 1:1), BT_REVIEW §7 Phase 0 (Arduino firmware
> watchdog — firmware side already implemented, see protocol notes below).

---

## 1. Current state (what is being ported)

### 1.1 The serial bridge — `scripts/arduino_bridge.py`

Node name `arduino_bridge`, single-threaded rclpy executor plus one
daemon **read thread** for the serial RX path. Structure:

| Piece | Behavior |
|---|---|
| Serial open | `port` (default `/dev/ttyUSB0`), `baudrate` (default 115200), 1 s read timeout. On failure: log error, exit (no retry loop). |
| TX timer | 10 Hz (hardcoded `0.1` — see §4.3 about the `publish_rate` param) sends the command frame. |
| RX thread | Blocking-ish loop: if bytes waiting → `readline()`, parse, publish; else sleep 10 ms. Any parse error → warn log, keep running. |
| Mow watchdog | Param `mow_cmd_timeout_s` (default **1.5 s**). If enable==1 but no `mowMotorEN_cmd` heard within the timeout → force enable 0, log error once, log info on resume. Commanders keep-alive at 2 Hz. |
| RPM guard | If enable==1 but `mowMotorRpmSet == 0` → send effective enable 0 (prevents blade start at 0 RPM before the setpoint topic arrives). |
| **F30 e-stop blade gate** | While `eStop_state == 1` (from RX frame field 0): force effective enable 0 + warn once, set latch. After e-stop release the latch keeps the blade off until the commander is seen dropping enable (explicit false OR the watchdog zeroing it); then info log + latch clears. See `PLAN_ESTOP_BLADE_GATE.md` for the four field scenarios — the C++ port must pass all four. |
| hoverBtnR1 pulse | `hoverBtnR1_pulse` (Bool): a `True` sets `hoverBtnR1_state = 1` and starts/restarts a **0.2 s one-shot timer** that drops it back to 0. Implemented by cancel+destroy+recreate of an rclpy timer. |

### 1.2 Bridge ROS interface (must be preserved EXACTLY — names, types, QoS depth 10)

Subscriptions (commands in):

| Topic | Type | Effect |
|---|---|---|
| `mowMotorEN_cmd` | `std_msgs/Bool` | blade enable + feeds the mow watchdog timestamp |
| `mowMotorRPM_set_cmd` | `std_msgs/Int32` | blade RPM setpoint |
| `hoverBtnR1_cmd` | `std_msgs/Bool` | hoverboard power button relay, level-controlled |
| `hoverBtnR1_pulse` | `std_msgs/Bool` | one-shot 0.2 s pulse of the same relay |
| `varaReleR2_cmd` | `std_msgs/Bool` | spare relay |
| `lidarPWR_cmd` | `std_msgs/Bool` | lidar power MOSFET |

Publications (status out, one per RX frame at ~10 Hz):

| Topic | Type | RX frame field |
|---|---|---|
| `eStop_status` | `std_msgs/Bool` | 0 |
| `mowMotorALM_status` | `std_msgs/Bool` | 1 |
| `bumperFront_status` | `std_msgs/Bool` | 2 |
| `mowMotorEN_status` | `std_msgs/Bool` | 3 |
| `mowMotorRPM_set_status` | `std_msgs/Int32` | 4 |
| `mowMotorRPM_FB_status` | `std_msgs/Int32` | 5 |
| `mowMotorCur_status` | `std_msgs/Float32` | 6 (float, e.g. `1.23`) |
| `hoverBtnR1_status` | `std_msgs/Bool` | 7 |
| `varaReleR2_status` | `std_msgs/Bool` | 8 |
| `lidarPWR_status` | `std_msgs/Bool` | 9 |

All topics are **relative** (no leading `/`) — keep it that way so any
future namespacing keeps working. Node name must stay `arduino_bridge` so
the existing `arduino_params.yaml` (keyed `arduino_bridge:`) still applies.

### 1.3 Serial protocol (defined by the Nano firmware — unchanged)

**Pi → Arduino** (10 Hz, newline-terminated ASCII CSV, 5 fields):

```
<mowMotorEN>,<mowMotorRpmSet>,<hoverBtnR1>,<varaReleR2>,<lidarPWR>\n
e.g. "1,3000,0,0,1\n"
```

Firmware accepts ≥5 comma-separated tokens, `atoi()`s them, drives pins,
and resets its own 2000 ms watchdog (`stopEverything()` on timeout).

**Arduino → Pi** (10 Hz, newline-terminated ASCII CSV, exactly 10 fields):

```
<eStop>,<!mowMotorALM>,<bumperFront>,<mowMotorEN>,<mowMotorRpmSet>,<mowMotorRpmFB>,<mowMotorCur:%.2f float>,<!hoverBtnR1>,<!varaReleR2>,<lidarPWR>\n
e.g. "0,0,0,1,3000,2987,1.42,1,0,1\n"
```

Notes the parser must survive (the Python version survives via broad
`except` — C++ must be equally tolerant, never crash):

- Firmware can also emit `WARNING:Watchdog triggered - Emergency Stop`
  lines → **skip any line that doesn't split into exactly 10 fields or
  fails numeric conversion**; warn-log and continue.
- Opening the port toggles DTR → **the Nano auto-resets** and spends
  ~0.5 s + calibration (~0.7 s total) in `setup()`. Early bytes may be
  garbage; the tolerant parser handles this, but see §4.4 (flush + settle
  delay on open — small robustness improvement over Python).
- Field 6 is a float with `.` decimal separator → parse with a
  locale-independent conversion (`std::from_chars` or `strtof` after
  ensuring `"C"` numeric locale; **do not** rely on default `std::stof`
  under a Finnish locale where `,` is the decimal separator).

### 1.4 Bridge parameters / config

`bringup/config/arduino_params.yaml` (installed to `hoverboard_driver`
share, passed by `arduino.launch.py`):

```yaml
arduino_bridge:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baudrate: 115200
    publish_rate: 0.1   # declared in yaml but NOT read by the Python node
```

`mow_cmd_timeout_s` (1.5) is declared in code only, not present in yaml.

### 1.5 The gamepad commander — `scripts/joy_to_arduino.py`

Node name `joy_to_arduino`. No serial — pure topic-to-topic logic that
turns raw `/joy` button states into the bridge's `*_cmd` topics. Launched
by `bringup/launch/joystick.launch.py` (NOT part of `bringup.launch.py`)
next to `joy_node` and `teleop_node`, all three sharing
`bringup/config/joystick.yaml`.

Interface (all QoS depth 10, relative names):

| Direction | Topic | Type |
|---|---|---|
| sub | `joy` | `sensor_msgs/Joy` |
| pub | `mowMotorEN_cmd` | `std_msgs/Bool` |
| pub | `mowMotorRPM_set_cmd` | `std_msgs/Int32` |
| pub | `hoverBtnR1_cmd` | `std_msgs/Bool` |
| pub | `varaReleR2_cmd` | `std_msgs/Bool` |
| pub | `lidarPWR_cmd` | `std_msgs/Bool` |

Parameters (defaults in code; yaml overrides in `joystick.yaml` under
`joy_to_arduino:` — note `lidarPWR_button` is currently code-default
only, not in the yaml):

| Param | Default | yaml | Meaning |
|---|---|---|---|
| `mowEnable_button` | 13 | 13 | blade enable button index |
| `mowRpmSET_value` | 100 | 2200 | RPM sent while blade enabled |
| `hoverBtnR1_button` | 7 | 7 | hoverboard power button index |
| `varaReleR2_button` | 6 | 6 | spare relay button index |
| `lidarPWR_button` | 3 | — | lidar power toggle button index |

Behavior to port 1:1:

- **Dynamic reconfigure**: `add_on_set_parameters_callback` live-updates
  all five params (`ros2 param set` works at runtime); each change is
  info-logged. C++: `add_on_set_parameters_callback` returning
  `rcl_interfaces::msg::SetParametersResult`.
- **Blade enable is edge-triggered**: `mowMotorEN_cmd` +
  `mowMotorRPM_set_cmd` publish only on button state CHANGE (press →
  `true` + configured RPM; release → `false` + `0`). Publishing every joy
  tick would flood the topic and fight the autonomous keep-alive.
- **2 Hz keep-alive timer** (0.5 s): while the blade button is held,
  republish RPM **then** enable — required because the bridge's
  `mow_cmd_timeout_s` watchdog (1.5 s) kills the blade without a
  keep-alive; RPM-before-enable so the bridge never sees enable=1 with
  rpm=0 (same rationale as `MowMotorController`).
- **hoverBtnR1 / varaReleR2 are level-published every joy tick** (20 Hz
  at the joy node's `autorepeat_rate`) as the current pressed state.
- **lidarPWR is a toggle on rising edge** (press flips the state,
  holding does nothing), with an info log per toggle.
- **Bounds guard**: callback bails out unless
  `len(msg.buttons) > mowEnable_idx`. NOTE: the Python only checks the
  mow index — an out-of-range `hoverBtnR1_button` etc. would raise
  IndexError. In C++ this would be UB, so the port bounds-checks **all
  four** indices (§4.9).

---

## 2. New package: `mowbot_arduino_bridge`

- **Location:** `/home/ros-pi/pi_ws/src/mowbot_arduino_bridge` (sibling of
  the other packages, own git repo — matches the `mowbot_*` naming already
  used by `mowbot_mqtt_bridge`).
- **Contents:** two executables — `arduino_bridge_node` and
  `joy_to_arduino_node`. They belong together: joy_to_arduino publishes
  exclusively to the bridge's command topics, and they share the
  "Arduino substation" domain.
- **Type:** `ament_cmake`, C++17, ROS 2 Jazzy.
- **Language:** all code comments, log strings, and docs in **English**.
  The Finnish comments in the Python originals are translated (they carry
  real design rationale — keep the content, translate the words). Topic
  and parameter names (`varaReleR2_cmd`, `mowRpmSET_value`, …) are public
  interface and stay exactly as-is.
- **Serial I/O:** plain **POSIX termios** (`open`/`tcsetattr`/`read`/
  `write`) wrapped in a small RAII class — zero external dependencies
  (no `libserial`, no `serial_driver`/`io_context` stack; neither is
  installed on the Pi and pulling them in buys nothing for one 115200-baud
  line-oriented port).

### 2.1 File tree

```
mowbot_arduino_bridge/
├── package.xml                  # format 3; deps: rclcpp, std_msgs, sensor_msgs, rcl_interfaces
├── CMakeLists.txt
├── LICENSE                      # match hoverboard_driver's license choice
├── README.md                    # protocol spec (§1.3), both nodes' topics/params, pointer to firmware repo
├── .gitignore                   # build/ install/ log/ .vscode/ etc.
├── include/mowbot_arduino_bridge/
│   ├── serial_port.hpp          # RAII termios wrapper
│   ├── arduino_bridge_node.hpp  # bridge node class declaration
│   └── joy_to_arduino_node.hpp  # gamepad commander node class declaration
├── src/
│   ├── serial_port.cpp
│   ├── arduino_bridge_node.cpp
│   ├── arduino_bridge_main.cpp  # main(): init, spin, shutdown
│   ├── joy_to_arduino_node.cpp
│   └── joy_to_arduino_main.cpp
├── config/
│   └── arduino_params.yaml      # moved here (new canonical home, §3.2)
└── launch/
    └── arduino_bridge.launch.py # optional standalone launch (bench tests without bringup)
```

(`joy_to_arduino` brings no config file of its own — its params live in
`hoverboard_driver`'s shared `joystick.yaml`, see §3.4.)

### 2.2 `serial_port.hpp/.cpp` — `SerialPort` class

```cpp
class SerialPort {
public:
  // throws std::runtime_error with errno text on failure
  void open(const std::string & device, int baudrate);
  void close();                                   // idempotent; called by dtor
  bool isOpen() const;
  void writeLine(const std::string & line);       // full-write loop, throws on error
  // Blocking read of one '\n'-terminated line, minus the terminator.
  // Returns false on timeout (no complete line yet). Internal byte buffer
  // carries partial lines across calls. CR stripped ("\r\n" tolerant).
  bool readLine(std::string & line, std::chrono::milliseconds timeout);
private:
  int fd_ = -1;
  std::string rx_buffer_;
};
```

termios setup: raw mode (`cfmakeraw`), 8N1, `B115200` via `cfsetispeed`/
`cfsetospeed` (map the int param through a baud-constant switch; reject
unsupported rates), no flow control, `VMIN=0/VTIME` or `poll()` for the
read timeout, `O_NOCTTY`. After open: 50 ms settle + `tcflush(TCIOFLUSH)`
(§4.4).

### 2.3 `arduino_bridge_node.hpp/.cpp` — `ArduinoBridgeNode : rclcpp::Node`

**Members** mirror the Python state 1:1:

- `SerialPort serial_;`
- Command state: `mow_en_`, `mow_rpm_set_`, `hover_btn_r1_`,
  `vara_rele_r2_`, `lidar_pwr_` (all `int`), `estop_state_` (written by RX
  thread, read by TX timer → `std::atomic<int>`).
- Watchdog: `mow_cmd_timeout_` (double, s), `last_mow_en_cmd_time_`
  (`std::optional<steady_clock::time_point>` — use the **steady** clock,
  the Python uses `time.monotonic()`), `mow_watchdog_tripped_` (bool).
- E-stop gate: `estop_blade_latched_` (bool).
- Pulse: `hover_btn_r1_pulse_timer_` (`rclcpp::TimerBase::SharedPtr`),
  duration constant 0.2 s.
- RX: `std::thread read_thread_;` + `std::atomic<bool> running_;`
- One mutex `state_mutex_` guarding the command-state ints that both the
  subscription callbacks and the TX timer touch. (Callbacks and timer run
  on the same single-threaded executor so they don't race each other, but
  taking the mutex is cheap and makes the design safe if anyone ever swaps
  in a MultiThreadedExecutor. `estop_state_` crosses the RX-thread
  boundary and is the one genuinely racy field — atomic regardless.)

**Constructor order** (mirrors Python):

1. Declare + read params: `port`, `baudrate`, `mow_cmd_timeout_s` (1.5),
   **`publish_rate` (0.1)** — now actually honored, see §4.3.
2. Open serial; on failure log fatal and `throw` → `main()` catches,
   returns non-zero (same "fail fast, let the service restart" behavior
   as the Python `raise SystemExit`).
3. Create the 6 subscriptions and 10 publishers (QoS depth 10, defaults).
4. Create TX wall-timer at `publish_rate`.
5. Start RX thread.

**Destructor / shutdown:** `running_ = false`, `join()` the RX thread
(readLine's timeout bounds the join wait), close port. Register nothing
with `rclcpp::on_shutdown` — plain RAII from `main()` is enough.

**TX path — `sendToArduino()`** (port of `send_to_arduino`, keep the
logic and the log messages 1:1):

1. Mow watchdog check (steady clock, timeout, trip once + error log,
   resume info log happens in the enable callback).
2. RPM guard (`effective_en = 0` when setpoint is 0).
3. F30 e-stop blade gate + latch (exact port of the Python block,
   including the warn/info log texts — the field-test plan in
   `PLAN_ESTOP_BLADE_GATE.md` greps for them; they are already English).
4. Format the 5-field frame with `snprintf`/`std::format` and
   `serial_.writeLine()`. Write errors: warn-log, don't crash (a
   disconnected USB adapter shouldn't take the node down mid-mission —
   see §4.5 for the reconnect decision).

**RX path — `readLoop()`** (port of `read_from_arduino`):

```
while (running_ && rclcpp::ok()):
    if (!serial_.readLine(line, 100ms)): continue;   // timeout == python's 10 ms sleep, no busy-wait
    split on ','; if fields != 10 → warn (throttled) + continue
    parse each field (from_chars); any failure → warn (throttled) + continue
    estop_state_.store(field[0])
    publish the 10 messages
```

Publishing from a non-executor thread is safe in rclcpp (publishers are
thread-safe); no extra executor needed.

**Pulse handling — `hoverBtnR1PulseCallback()`:** on `data==true`, set
state 1 and (re)create a one-shot timer. rclcpp has native one-shots:
`create_wall_timer` + `timer->cancel()` inside the callback, or Jazzy's
`create_timer(...)` with manual cancel — replicate "new pulse restarts
the 0.2 s window" by cancelling/resetting the existing timer first
(rclcpp `TimerBase::reset()` makes this cleaner than the Python
destroy/recreate dance).

### 2.4 `joy_to_arduino_node.hpp/.cpp` — `JoyToArduinoNode : rclcpp::Node`

Straight port of §1.5 — no threads, no serial, just callbacks:

- **Members:** the five param values (plain ints, single-threaded
  executor → no locking needed), `prev_mow_en_` (bool),
  `prev_lidar_pwr_` (bool), `lidar_pwr_state_` (bool), the five
  publishers, the `joy` subscription, the 0.5 s keep-alive wall-timer,
  and the `OnSetParametersCallbackHandle` for dynamic reconfigure.
- **`joyCallback(const sensor_msgs::msg::Joy &)`:**
  1. Bounds-check **all four** button indices against
     `msg.buttons.size()` (delta vs. Python, §4.9); skip the frame with a
     throttled warn if any is out of range.
  2. Blade: edge-triggered publish of enable + RPM (press → `true` +
     `mowRpmSET_value`, release → `false` + `0`). Publish **RPM before
     enable** (delta §4.10 — normalizes the ordering the keep-alive
     already uses; the Python joy_callback published enable first).
  3. hoverBtnR1 / varaReleR2: publish current level every tick.
  4. lidarPWR: rising-edge toggle + info log.
- **`mowKeepalive()`** (0.5 s timer): while `prev_mow_en_` is true,
  republish RPM then enable — feeds the bridge's 1.5 s watchdog exactly
  like today.
- **`parametersCallback()`:** accept updates to the five params,
  info-log each change (English), return `successful=true`.

### 2.5 `package.xml` / `CMakeLists.txt`

- `package.xml`: format 3, `<buildtool_depend>ament_cmake`,
  `<depend>rclcpp</depend>`, `<depend>std_msgs</depend>`,
  `<depend>sensor_msgs</depend>`, `<depend>rcl_interfaces</depend>`,
  `<test_depend>ament_lint_auto/ament_lint_common</test_depend>`.
  Maintainer = Tuomas (leppanen.tuomas@gmail.com).
- `CMakeLists.txt`: C++17, `-Wall -Wextra -Wpedantic`, two targets:
  - `add_executable(arduino_bridge_node src/arduino_bridge_main.cpp
    src/arduino_bridge_node.cpp src/serial_port.cpp)`
  - `add_executable(joy_to_arduino_node src/joy_to_arduino_main.cpp
    src/joy_to_arduino_node.cpp)`
  both installed to `lib/${PROJECT_NAME}`; install `launch/` and
  `config/` to `share/${PROJECT_NAME}`.
- Executable names: **`arduino_bridge_node`** / **`joy_to_arduino_node`**
  (the nodes inside still name themselves `arduino_bridge` /
  `joy_to_arduino` so the existing yaml keys keep applying).

### 2.6 Git repo

```bash
cd /home/ros-pi/pi_ws/src/mowbot_arduino_bridge
git init -b main
git add -A && git commit   # "Initial C++ port of arduino_bridge + joy_to_arduino"
```

`.gitignore`: `build/`, `install/`, `log/`, `.vscode/`, `*.pyc`,
`__pycache__/`. Remote (GitHub etc.) can be added later — not blocking.

---

## 3. Launch & config integration (this repo, `hoverboard_driver`)

### 3.1 `bringup/launch/arduino.launch.py` — repoint the Node

`bringup.launch.py` is **untouched** (it just includes
`arduino.launch.py` from the `hoverboard_driver` share). Edit only the
included file (and translate its Finnish comments to English while
there):

```python
config = os.path.join(
    get_package_share_directory('mowbot_arduino_bridge'),   # was hoverboard_driver
    'config', 'arduino_params.yaml')

arduino_node = Node(
    package='mowbot_arduino_bridge',       # was hoverboard_driver
    executable='arduino_bridge_node',      # was arduino_bridge.py
    name='arduino_bridge',
    output='screen',
    parameters=[config])
```

### 3.2 `arduino_params.yaml` moves with the bridge

Canonical `arduino_params.yaml` moves to
`mowbot_arduino_bridge/config/`. Add `mow_cmd_timeout_s: 1.5`, keep
`publish_rate: 0.1` (now honored), translate the comments to English.
Delete `bringup/config/arduino_params.yaml` from this repo **in the same
commit** that repoints the launch file, so there's never a stale
duplicate to edit by mistake.

### 3.3 `bringup/launch/joystick.launch.py` — repoint joy_to_arduino

`joy_node` and `teleop_node` entries stay untouched; only:

```python
joy_to_arduino_node = Node(
    package='mowbot_arduino_bridge',       # was hoverboard_driver
    executable='joy_to_arduino_node',      # was joy_to_arduino.py
    name='joy_to_arduino',
    parameters=[joy_params])               # unchanged: shared joystick.yaml
```

### 3.4 `joystick.yaml` STAYS in `hoverboard_driver`

It configures three nodes (`joy_node`, `teleop_node`, `joy_to_arduino`)
and the first two remain launched from this repo — splitting the file
would be worse than the cross-package param file. Passing another
package's yaml to the C++ node works exactly as before (it's just a file
path). Translate its Finnish comments to English in passing. Optional
tidy-up: add the missing `lidarPWR_button: 3` so all five params are
visible in one place.

### 3.5 Retire both Python nodes from `hoverboard_driver`

Same commit as 3.1–3.3:

- `CMakeLists.txt`: drop `scripts/arduino_bridge.py` **and**
  `scripts/joy_to_arduino.py` from `install(PROGRAMS ...)`.
- Delete both script files (git history keeps them; plan docs reference
  them by name, which is fine — they describe history).
- Remove `<exec_depend>python3-serial</exec_depend>` — verified 2026-07-10:
  `arduino_bridge.py` is the **only** script in this repo importing
  `serial`.
- `package.xml` of `hoverboard_driver`: add
  `<exec_depend>mowbot_arduino_bridge</exec_depend>` so the launch files'
  cross-package references are a declared dependency.

---

## 4. Design decisions & deltas vs. the Python nodes

Everything not listed here is a 1:1 port.

1. **Package name `mowbot_arduino_bridge`** — `arduino_bridge` alone is
   too generic for a standalone repo; `mowbot_` prefix matches
   `mowbot_mqtt_bridge`. Contains both nodes (§2).
2. **Language/runtime**: C++17, termios, `std::thread` RX loop — same
   architecture as Python (timer TX + thread RX for the bridge; pure
   callbacks for joy_to_arduino), no architectural rewrite hiding behind
   the port.
3. **`publish_rate` becomes real**: the yaml already carries it; the C++
   bridge declares and uses it (default 0.1 s). Behavior at the shipped
   value is identical to today.
4. **Flush + settle on open** (new, small): 50 ms delay +
   `tcflush` after opening swallows the Nano auto-reset garbage instead
   of relying purely on parser tolerance. No protocol impact.
5. **Serial write errors don't kill the bridge** (Python: unhandled
   exception in the timer would). Warn-throttled log instead.
   **Auto-reconnect is explicitly OUT of scope** for this port ("same
   functionality") — noted as a candidate follow-up. On persistent write
   failure the node keeps trying at 10 Hz; the Arduino-side 2 s watchdog
   already guarantees the blade stops when frames stop arriving.
6. **Throttled warn logs** in the RX parser (Python warns per bad line;
   at 10 Hz garbage that floods journald). Use
   `RCLCPP_WARN_THROTTLE(..., 5000, ...)`.
7. **Locale-safe float parsing** for `mowMotorCur` (§1.3).
8. **English everywhere** (user decision 2026-07-10): all Finnish
   comments and log strings are translated to English in the C++ port —
   the comments' design rationale is preserved, just translated. The F30
   gate + mow watchdog safety logs are already English and are kept
   **verbatim** (existing test plans grep for them). Topic names, node
   names, and parameter names — including Finnish-derived ones like
   `varaReleR2_cmd` — are public interface and are NOT renamed.
9. **joy_to_arduino bounds-checks all four button indices** (Python only
   checked `mowEnable_button`; an out-of-range index for the other three
   would have thrown IndexError per tick — in C++ it would be UB, so the
   port checks all of them and warn-throttles on a misconfigured index).
10. **joy_to_arduino publishes RPM before enable on press** (the Python
    joy_callback published enable first, while its own keep-alive and
    `MowMotorController` publish RPM first for the bridge's RPM-guard
    reason). Harmless either way — the bridge's 10 Hz TX tick sees both —
    but the port normalizes to RPM-first to match the documented
    rationale.

---

## 5. Implementation order

1. **Scaffold** the package (tree in §2.1, package.xml, CMakeLists,
   .gitignore, two empty nodes that build). `git init` + first commit.
2. **SerialPort** class + a tiny standalone test binary or gtest with
   `socat`-created PTY pair (`socat -d -d pty,raw,echo=0 pty,raw,echo=0`)
   — verifies readLine framing, timeout, partial-line carry-over without
   hardware.
3. **Bridge node skeleton**: params, pubs/subs, TX timer sending frames,
   RX thread publishing — protocol only, no safety logic yet.
   Bench-verify against the real Nano: `ros2 topic echo /eStop_status`,
   command echo via `ros2 topic pub`.
4. **Bridge safety logic**: mow watchdog → RPM guard → F30 e-stop
   gate+latch → hoverBtnR1 pulse. Port each with its log messages.
5. **joy_to_arduino node**: params + dynamic reconfigure → joy callback
   (edge/level/toggle logic) → keep-alive timer. Bench-verify with
   `joy_node` + gamepad, or by publishing `sensor_msgs/Joy` by hand.
6. **Integration**: edits in `hoverboard_driver` (§3), `colcon build
   --packages-select mowbot_arduino_bridge hoverboard_driver`, full
   `bringup.launch.py` + `joystick.launch.py` smoke test.
7. **Cleanup + docs**: README, final commits in both repos.

---

## 6. Test plan

### 6.1 Bridge (Nano on USB, blade motor disconnected or safe)

- [ ] Node starts via `ros2 launch hoverboard_driver arduino.launch.py`;
      port from yaml is honored (try a wrong port → clean fatal exit).
- [ ] All 10 `_status` topics publish at ~10 Hz with sane values
      (`ros2 topic hz`, `ros2 topic echo`).
- [ ] `ros2 topic pub /lidarPWR_cmd std_msgs/Bool "data: true"` →
      lidar MOSFET pin toggles, `lidarPWR_status` follows.
- [ ] `hoverBtnR1_pulse` → relay closes for 0.2 s, releases; repeated
      pulses restart the window.
- [ ] **Mow watchdog**: publish `mowMotorEN_cmd true` + RPM once, stop
      publishing → within ~1.5 s the TX frame drops enable, error logged
      once; resume publishing → info log, enable returns.
- [ ] **RPM guard**: enable=true with no RPM setpoint → frame keeps en=0.
- [ ] **F30 e-stop gate — all four scenarios** from
      `PLAN_ESTOP_BLADE_GATE.md`: gamepad held-through (latch holds),
      web/mission instant clear, watchdog-fallback clear, e-stop with no
      blade running (no spurious latch).
- [ ] Unplug USB mid-run → node logs throttled warnings, doesn't crash;
      replug does NOT auto-recover (documented, §4.5) → restart service.
- [ ] Kill the node → Arduino's own 2 s watchdog fires
      (`stopEverything()`, relays to safe state).
- [ ] 30 min soak: no RX parse warnings beyond startup, stable memory/CPU
      (compare `top` against the Python node's footprint for the README).

### 6.2 joy_to_arduino (with `joystick.launch.py` + gamepad)

- [ ] Params load from `joystick.yaml` (RPM 2200, buttons 13/7/6; lidar
      button code-default 3).
- [ ] Blade button press → single `mowMotorEN_cmd true` +
      `mowMotorRPM_set_cmd 2200` (RPM first), then 2 Hz keep-alive while
      held; release → single `false` + `0`, keep-alive stops.
- [ ] Hold blade button > 1.5 s → bridge watchdog does NOT trip (keep-alive
      works end-to-end).
- [ ] hoverBtnR1 / varaReleR2 buttons: `_cmd` topics track the level at
      joy rate.
- [ ] lidarPWR button: each press toggles `lidarPWR_cmd`, hold does not
      re-toggle.
- [ ] `ros2 param set /joy_to_arduino mowRpmSET_value 1800` at runtime →
      next press publishes 1800 (dynamic reconfigure works).
- [ ] Misconfigured button index (e.g. `hoverBtnR1_button: 99`) →
      throttled warn, no crash.
- [ ] **F30 regression with the C++ pair**: blade held on gamepad +
      physical e-stop → blade stops and stays latched until button
      release (the exact field scenario that motivated the gate).

Field: re-run the F30 field test owed from `PLAN_ESTOP_BLADE_GATE.md` on
the C++ nodes (they supersede the Python implementation that test
targeted).

---

## 7. Deployment notes

- The Python nodes were symlink-installed (edit + service restart, no
  rebuild). The C++ nodes **require `colcon build` after every change** —
  document in README; `--symlink-install` doesn't help compiled targets.
- Rollout: build both packages → restart `mowbot-launch-bringup` →
  verify `_status` topics + gamepad path → only then commit the
  Python-node removal (keep the removal as a separate final commit so
  rollback is `git revert` + restart, no rebuild of the old Python path).
- Update `plans/PLAN_ESTOP_BLADE_GATE.md` status header to point at the
  new implementation location once ported.

## 8. Open questions (non-blocking, defaults chosen)

- License for the new repo — defaulting to whatever
  `hoverboard_driver/package.xml` declares.
- GitHub remote now or later — later; local repo is enough to start.
- ~~English vs. Finnish~~ — **decided 2026-07-10: English** for all
  comments/logs/docs in the new repo (§4.8); safety-log strings that
  existing test plans grep for are kept verbatim (already English).
