// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hoverboard_driver/hoverboard_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hoverboard_driver
{

  namespace
  {
    constexpr double RPM_TO_RADPS = 2.0 * M_PI / 60.0; // 0.10472
    constexpr double A2BIT_CONV = 50.0;                // firmware iq raw units per amp
    constexpr int16_t MOTOR_FLAG_ENABLE = 0x0001;      // SerialCommand.flags bit0

    double clampd(double v, double lo, double hi)
    {
      return v < lo ? lo : (v > hi ? hi : v);
    }
  } // namespace

  // ========================== helper node ==========================

  hoverboard_driver_node::hoverboard_driver_node(SharedState *state)
      : Node("hoverboard_driver_node"), state_(state)
  {
    declare_parameter("motors_enabled", false);
    declare_parameter("auto_disable_timeout", 120.0);
    declare_parameter("publish_debug", true);
    state_->motors_enabled = get_parameter("motors_enabled").as_bool();
    state_->auto_disable_timeout = get_parameter("auto_disable_timeout").as_double();
    state_->publish_debug = get_parameter("publish_debug").as_bool();

    callback_handle_ = add_on_set_parameters_callback(
        std::bind(&hoverboard_driver_node::parametersCallback, this, std::placeholders::_1));

    const rclcpp::QoS debug_qos(3);
    const auto latched_qos = rclcpp::QoS(1).transient_local();

    vel_pub_[left_wheel] = create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/velocity", debug_qos);
    vel_pub_[right_wheel] = create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/velocity", debug_qos);
    pos_pub_[left_wheel] = create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/position", debug_qos);
    pos_pub_[right_wheel] = create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/position", debug_qos);
    cmd_pub_[left_wheel] = create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/cmd", debug_qos);
    cmd_pub_[right_wheel] = create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/cmd", debug_qos);
    curr_pub_[left_wheel] = create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/dc_current", debug_qos);
    curr_pub_[right_wheel] = create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/dc_current", debug_qos);
    iq_pub_[left_wheel] = create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/iq_current", debug_qos);
    iq_pub_[right_wheel] = create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/iq_current", debug_qos);
    voltage_pub_ = create_publisher<std_msgs::msg::Float64>("hoverboard/battery_voltage", debug_qos);
    temp_pub_ = create_publisher<std_msgs::msg::Float64>("hoverboard/temperature", debug_qos);

    connected_pub_ = create_publisher<std_msgs::msg::Bool>("hoverboard/connected", latched_qos);
    motors_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("hoverboard/motors_enabled", latched_qos);
    fw_timeout_pub_ = create_publisher<std_msgs::msg::Bool>("hoverboard/firmware_serial_timeout", latched_qos);
    error_pub_[left_wheel] = create_publisher<std_msgs::msg::UInt8>("hoverboard/left_wheel/error", latched_qos);
    error_pub_[right_wheel] = create_publisher<std_msgs::msg::UInt8>("hoverboard/right_wheel/error", latched_qos);

    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&hoverboard_driver_node::timerCallback, this));
  }

  void hoverboard_driver_node::timerCallback()
  {
    SharedState snap;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      snap.voltage = state_->voltage;
      snap.temperature = state_->temperature;
      for (int i = 0; i < 2; i++)
      {
        snap.dc_curr[i] = state_->dc_curr[i];
        snap.iq_curr[i] = state_->iq_curr[i];
        snap.vel[i] = state_->vel[i];
        snap.pos[i] = state_->pos[i];
        snap.cmd[i] = state_->cmd[i];
        snap.motor_error[i] = state_->motor_error[i];
      }
      snap.fw_motors_enabled = state_->fw_motors_enabled;
      snap.fw_serial_timeout = state_->fw_serial_timeout;
      snap.connected = state_->connected;
    }

    if (state_->publish_debug.load())
    {
      std_msgs::msg::Float64 f;
      for (int i = 0; i < 2; i++)
      {
        f.data = snap.vel[i];
        vel_pub_[i]->publish(f);
        f.data = snap.pos[i];
        pos_pub_[i]->publish(f);
        f.data = snap.cmd[i];
        cmd_pub_[i]->publish(f);
        f.data = snap.dc_curr[i];
        curr_pub_[i]->publish(f);
        f.data = snap.iq_curr[i];
        iq_pub_[i]->publish(f);
      }
      f.data = snap.voltage;
      voltage_pub_->publish(f);
      f.data = snap.temperature;
      temp_pub_->publish(f);
    }

    std_msgs::msg::Bool b;
    std_msgs::msg::UInt8 u;
    if (first_status_pub_ || snap.connected != last_connected_)
    {
      b.data = snap.connected;
      connected_pub_->publish(b);
      last_connected_ = snap.connected;
    }
    if (first_status_pub_ || snap.fw_motors_enabled != last_fw_enabled_)
    {
      b.data = snap.fw_motors_enabled;
      motors_enabled_pub_->publish(b);
      last_fw_enabled_ = snap.fw_motors_enabled;
    }
    if (first_status_pub_ || snap.fw_serial_timeout != last_fw_timeout_)
    {
      b.data = snap.fw_serial_timeout;
      fw_timeout_pub_->publish(b);
      last_fw_timeout_ = snap.fw_serial_timeout;
    }
    for (int i = 0; i < 2; i++)
    {
      if (first_status_pub_ || snap.motor_error[i] != last_error_[i])
      {
        u.data = snap.motor_error[i];
        error_pub_[i]->publish(u);
        last_error_[i] = snap.motor_error[i];
        if (snap.motor_error[i] != 0)
        {
          RCLCPP_WARN(get_logger(), "%s motor error code: %u (1=hall disconnected, 2=hall short, 4=blocked)",
                      i == left_wheel ? "Left" : "Right", snap.motor_error[i]);
        }
      }
    }
    first_status_pub_ = false;
  }

  rcl_interfaces::msg::SetParametersResult hoverboard_driver_node::parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
      if (param.get_name() == "motors_enabled")
      {
        state_->motors_enabled = param.as_bool();
        RCLCPP_INFO(get_logger(), "motors_enabled set to %s", param.as_bool() ? "true" : "false");
      }
      else if (param.get_name() == "auto_disable_timeout")
      {
        if (param.as_double() <= 0.0)
        {
          result.successful = false;
          result.reason = "auto_disable_timeout must be > 0";
          return result;
        }
        state_->auto_disable_timeout = param.as_double();
        RCLCPP_INFO(get_logger(), "auto_disable_timeout set to %.1f s", param.as_double());
      }
      else if (param.get_name() == "publish_debug")
      {
        state_->publish_debug = param.as_bool();
      }
    }
    return result;
  }

  // ========================== hardware interface ==========================

  hoverboard_driver::~hoverboard_driver()
  {
    if (executor_ && spin_thread_.joinable())
    {
      // cancel() issued before the thread enters spin() would be overwritten by
      // spin()'s spinning.exchange(true) and join() would hang; wait until the
      // executor is actually spinning (or the thread already exited) first.
      while (!executor_->is_spinning() && !spin_done_.load())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      executor_->cancel();
    }
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
  }

  hardware_interface::CallbackReturn hoverboard_driver::on_init(
      const hardware_interface::HardwareComponentInterfaceParams &info)
  {
    if (hardware_interface::HardwareComponentInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Read parameters from the ros2_control xacro; fail cleanly if missing/invalid.
    for (const char *key : {"wheel_radius", "max_velocity", "device"})
    {
      if (info_.hardware_parameters.count(key) == 0)
      {
        RCLCPP_FATAL(rclcpp::get_logger("hoverboard_driver"),
                     "Missing hardware parameter '%s' in ros2_control description", key);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    try
    {
      wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
      max_velocity_radps_ = std::stod(info_.hardware_parameters["max_velocity"]) / wheel_radius_;
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL(rclcpp::get_logger("hoverboard_driver"),
                   "Invalid numeric hardware parameter: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (wheel_radius_ <= 0.0 || max_velocity_radps_ <= 0.0)
    {
      RCLCPP_FATAL(rclcpp::get_logger("hoverboard_driver"),
                   "wheel_radius and max_velocity must be > 0");
      return hardware_interface::CallbackReturn::ERROR;
    }
    port_ = info_.hardware_parameters["device"];

    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("HoverBoardSystemHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("HoverBoardSystemHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("HoverBoardSystemHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("HoverBoardSystemHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("HoverBoardSystemHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // Helper node on its own executor thread: parameters and all publishing
    // happen there, never in the ros2_control read/write path.
    hardware_publisher_ = std::make_shared<hoverboard_driver_node>(&shared_state_);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(hardware_publisher_);
    spin_thread_ = std::thread([this]()
                               {
                                 executor_->spin();
                                 spin_done_ = true;
                               });

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> hoverboard_driver::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> hoverboard_driver::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn hoverboard_driver::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"), "Using port %s", port_.c_str());

    low_wrap_ = static_cast<int>(ENCODER_LOW_WRAP_FACTOR * (ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN);
    high_wrap_ = static_cast<int>(ENCODER_HIGH_WRAP_FACTOR * (ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN);

    // Reset all per-session state
    for (int i = 0; i < 2; i++)
    {
      last_wheelcount_[i] = 0;
      mult_[i] = 0;
      last_pos_[i] = 0.0;
      accum_pos_[i] = 0.0;
      hw_positions_[i] = 0.0;
      hw_velocities_[i] = 0.0;
      hw_commands_[i] = 0.0;
    }
    first_encoder_pass_ = true;
    have_valid_frame_ = false;
    fw_enabled_rt_ = false;
    fw_connected_rt_ = false;
    arm_request_ = false;
    prev_allowed_ = false;
    have_cmd_activity_ = false;
    msg_len_ = 0;
    prev_byte_ = 0;

    if ((port_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("hoverboard_driver"),
                   "Cannot open serial port %s to hoverboard", port_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // CONFIGURE THE UART -- connecting to the board
    // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    struct termios options;
    tcgetattr(port_fd_, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD; //<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(port_fd_, TCIFLUSH);
    tcsetattr(port_fd_, TCSANOW, &options);

    RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"),
                "Successfully activated! Motors start disarmed (standby); set parameter "
                "'motors_enabled' on hoverboard_driver_node to arm.");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn hoverboard_driver::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    if (port_fd_ != -1)
    {
      // Best-effort disarm frame so the board drops to standby immediately
      // (~10 ms) instead of waiting out its 0.8 s serial-timeout failsafe.
      SerialCommand stop;
      stop.start = static_cast<uint16_t>(START_FRAME);
      stop.steer = 0;
      stop.speed = 0;
      stop.flags = 0;
      stop.checksum = static_cast<uint16_t>(stop.start ^ stop.steer ^ stop.speed ^ stop.flags);
      if (::write(port_fd_, &stop, sizeof(stop)) != static_cast<ssize_t>(sizeof(stop)))
      {
        RCLCPP_WARN(rclcpp::get_logger("hoverboard_driver"),
                    "Could not send disarm frame on deactivate; firmware timeout will disarm");
      }
      tcdrain(port_fd_);
      close(port_fd_);
      port_fd_ = -1;
    }
    arm_request_ = false;
    prev_allowed_ = false;

    {
      std::lock_guard<std::mutex> lock(shared_state_.mutex);
      shared_state_.connected = false;
      shared_state_.fw_motors_enabled = false;
    }

    RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type hoverboard_driver::read(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    if (port_fd_ != -1)
    {
      // Chunked read: drain the port with few syscalls instead of byte-at-a-time.
      uint8_t buf[256];
      int total = 0;
      while (total < 2048)
      {
        const ssize_t r = ::read(port_fd_, buf, sizeof(buf));
        if (r > 0)
        {
          for (ssize_t i = 0; i < r; i++)
          {
            protocol_recv(time, buf[i]);
          }
          total += static_cast<int>(r);
          continue;
        }
        if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
          RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("hoverboard_driver"), steady_clock_, 5000,
                                "Reading from serial %s failed: %s", port_.c_str(), std::strerror(errno));
        }
        break;
      }
    }

    // Connected = a checksum-valid frame within the last second (not just bytes).
    const bool connected =
        have_valid_frame_ && (time - last_valid_frame_).seconds() < 1.0;
    fw_connected_rt_ = connected;
    if (!connected)
    {
      fw_enabled_rt_ = false;
      // Zero the exported velocities: the firmware freewheels on its own serial
      // timeout, so the wheels coast to a stop. Serving the last in-motion value
      // would make diff_drive integrate phantom odometry for the whole outage.
      hw_velocities_[left_wheel] = 0.0;
      hw_velocities_[right_wheel] = 0.0;
    }

    {
      std::lock_guard<std::mutex> lock(shared_state_.mutex);
      shared_state_.connected = connected;
      if (!connected)
      {
        shared_state_.fw_motors_enabled = false;
        shared_state_.vel[left_wheel] = 0.0;
        shared_state_.vel[right_wheel] = 0.0;
      }
    }

    return hardware_interface::return_type::OK;
  }

  void hoverboard_driver::protocol_recv(const rclcpp::Time &time, uint8_t byte)
  {
    const uint16_t start_frame = (static_cast<uint16_t>(byte) << 8) | prev_byte_;

    if (start_frame == START_FRAME)
    {
      p_ = reinterpret_cast<uint8_t *>(&msg_);
      *p_++ = prev_byte_;
      *p_++ = byte;
      msg_len_ = 2;
    }
    else if (msg_len_ >= 2 && msg_len_ < sizeof(SerialFeedback))
    {
      *p_++ = byte;
      msg_len_++;
    }

    if (msg_len_ == sizeof(SerialFeedback))
    {
      const uint16_t checksum = static_cast<uint16_t>(
          msg_.start ^ msg_.cmd1 ^ msg_.cmd2 ^ msg_.speedR_meas ^ msg_.speedL_meas ^
          msg_.wheelR_cnt ^ msg_.wheelL_cnt ^ msg_.left_dc_curr ^ msg_.right_dc_curr ^
          msg_.iq_l ^ msg_.iq_r ^ msg_.batVoltage ^ msg_.boardTemp ^ msg_.cmdLed);

      if (msg_.start == START_FRAME && msg_.checksum == checksum)
      {
        // Convert RPM to rad/s; right motor is mirrored.
        hw_velocities_[left_wheel] = msg_.speedL_meas * RPM_TO_RADPS;
        hw_velocities_[right_wheel] = -msg_.speedR_meas * RPM_TO_RADPS;

        // Wheel positions from the raw modulo-9000 tick counters
        // (uses the previous last_valid_frame_ for board-restart detection,
        // so it must run before last_valid_frame_ is updated below).
        on_encoder_update(time, msg_.wheelR_cnt, msg_.wheelL_cnt);

        fw_enabled_rt_ = (msg_.cmdLed >> 8) & 0x01;

        {
          std::lock_guard<std::mutex> lock(shared_state_.mutex);
          shared_state_.voltage = msg_.batVoltage / 100.0;
          shared_state_.temperature = msg_.boardTemp / 10.0;
          shared_state_.dc_curr[left_wheel] = msg_.left_dc_curr / 100.0;
          shared_state_.dc_curr[right_wheel] = msg_.right_dc_curr / 100.0;
          shared_state_.iq_curr[left_wheel] = msg_.iq_l / A2BIT_CONV;
          shared_state_.iq_curr[right_wheel] = msg_.iq_r / A2BIT_CONV;
          shared_state_.vel[left_wheel] = hw_velocities_[left_wheel];
          shared_state_.vel[right_wheel] = hw_velocities_[right_wheel];
          shared_state_.pos[left_wheel] = hw_positions_[left_wheel];
          shared_state_.pos[right_wheel] = hw_positions_[right_wheel];
          shared_state_.motor_error[left_wheel] = msg_.cmdLed & 0x0F;
          shared_state_.motor_error[right_wheel] = (msg_.cmdLed >> 4) & 0x0F;
          shared_state_.fw_motors_enabled = fw_enabled_rt_;
          shared_state_.fw_serial_timeout = (msg_.cmdLed >> 9) & 0x01;
        }

        last_valid_frame_ = time;
        have_valid_frame_ = true;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("hoverboard_driver"), steady_clock_, 5000,
                             "Hoverboard checksum mismatch: %d vs %d (throttled 5 s)",
                             msg_.checksum, checksum);
      }
      msg_len_ = 0;
    }
    prev_byte_ = byte;
  }

  hardware_interface::return_type hoverboard_driver::write(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    if (port_fd_ == -1)
    {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("hoverboard_driver"), steady_clock_, 5000,
                            "Attempt to write on closed serial");
      return hardware_interface::return_type::ERROR;
    }

    const double cmd_l = hw_commands_[left_wheel];  // rad/s
    const double cmd_r = hw_commands_[right_wheel]; // rad/s
    const bool cmd_active = std::abs(cmd_l) > 1e-3 || std::abs(cmd_r) > 1e-3;
    const bool allowed = shared_state_.motors_enabled.load();

    // Arming policy: explicit enable arms immediately; auto-disable after
    // auto_disable_timeout of zero commands; silent re-arm on command resume.
    if (!allowed)
    {
      arm_request_ = false;
    }
    else
    {
      if (!prev_allowed_)
      {
        arm_request_ = true; // rising edge of motors_enabled: arm now
        last_cmd_activity_ = time;
        have_cmd_activity_ = true;
      }
      if (cmd_active)
      {
        last_cmd_activity_ = time;
        have_cmd_activity_ = true;
        arm_request_ = true;
      }
      else if (arm_request_ && have_cmd_activity_ &&
               (time - last_cmd_activity_).seconds() > shared_state_.auto_disable_timeout.load())
      {
        arm_request_ = false;
        RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"),
                    "Auto-disabling motors after %.0f s of zero commands (standby)",
                    shared_state_.auto_disable_timeout.load());
      }
    }
    prev_allowed_ = allowed;

    // Send zeros until the firmware confirms it armed (status bit in feedback):
    // guarantees the firmware's "inputs near zero" arming gate passes, and
    // covers board brownout mid-drive (enabled bit drops -> zeros -> re-arm).
    double set_l_rpm = 0.0;
    double set_r_rpm = 0.0;
    if (arm_request_ && fw_connected_rt_ && fw_enabled_rt_)
    {
      // Limit wheel speed by scaling BOTH wheels with a common factor so the
      // commanded curvature is preserved (independent clamping would distort arcs).
      double l = cmd_l;
      double r = cmd_r;
      const double mag = std::max(std::abs(l), std::abs(r));
      if (mag > max_velocity_radps_)
      {
        const double scale = max_velocity_radps_ / mag;
        l *= scale;
        r *= scale;
      }
      set_l_rpm = l / RPM_TO_RADPS;
      set_r_rpm = r / RPM_TO_RADPS;
      // Note: after a board brownout the re-arm steps the target from 0 back to
      // cruise in one frame; the firmware's own rate limiter + input low-pass
      // (~100 ms) smooth that step, so no driver-side ramp is applied.
    }

    // Firmware mixer inversion (STEER_COEFFICIENT 0.5, SPEED_COEFFICIENT 1.0):
    // cmdL = speed + 0.5*steer, cmdR = speed - 0.5*steer -> per-wheel passthrough.
    const double speed_d = clampd((set_l_rpm + set_r_rpm) / 2.0, -1000.0, 1000.0);
    const double steer_d = clampd(set_l_rpm - set_r_rpm, -1000.0, 1000.0);

    SerialCommand command;
    command.start = static_cast<uint16_t>(START_FRAME);
    command.steer = static_cast<int16_t>(std::lround(steer_d));
    command.speed = static_cast<int16_t>(std::lround(speed_d));
    command.flags = arm_request_ ? MOTOR_FLAG_ENABLE : 0;
    command.checksum = static_cast<uint16_t>(command.start ^ command.steer ^ command.speed ^ command.flags);

    const ssize_t rc = ::write(port_fd_, &command, sizeof(command));
    if (rc != static_cast<ssize_t>(sizeof(command)))
    {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("hoverboard_driver"), steady_clock_, 5000,
                            "Error writing to hoverboard serial port (rc=%zd)", rc);
    }

    {
      std::lock_guard<std::mutex> lock(shared_state_.mutex);
      shared_state_.cmd[left_wheel] = cmd_l;
      shared_state_.cmd[right_wheel] = cmd_r;
    }

    return hardware_interface::return_type::OK;
  }

  void hoverboard_driver::on_encoder_update(
      const rclcpp::Time &time, int16_t right_raw, int16_t left_raw)
  {
    const int16_t raw[2] = {left_raw, right_raw}; // both in [0, ENCODER_MAX)

    // Board-restart detection on the RAW wrapped counts (wrap-aware near-zero):
    // after a data gap, counters restarting near zero indicate a power cycle.
    if (!first_encoder_pass_ && have_valid_frame_ &&
        (time - last_valid_frame_).seconds() > 0.2)
    {
      bool near_zero = true;
      for (int i = 0; i < 2; i++)
      {
        near_zero = near_zero && (raw[i] <= 5 || raw[i] >= ENCODER_MAX - 5);
      }
      if (near_zero)
      {
        RCLCPP_WARN(rclcpp::get_logger("hoverboard_driver"),
                    "Hoverboard restart detected - preserving accumulated wheel positions");
        for (int i = 0; i < 2; i++)
        {
          mult_[i] = 0;
          last_wheelcount_[i] = raw[i];
          last_pos_[i] = raw[i]; // no delta on this frame
        }
      }
    }

    for (int i = 0; i < 2; i++)
    {
      // Wrap detection in the raw [0, ENCODER_MAX) domain (fix for the old
      // negate-before-unwrap bug that broke the right wheel every 9000 ticks).
      if (raw[i] < low_wrap_ && last_wheelcount_[i] > high_wrap_)
      {
        mult_[i]++;
      }
      else if (raw[i] > high_wrap_ && last_wheelcount_[i] < low_wrap_)
      {
        mult_[i]--;
      }
      last_wheelcount_[i] = raw[i];

      const double pos = raw[i] + mult_[i] * (ENCODER_MAX - ENCODER_MIN);
      if (first_encoder_pass_)
      {
        last_pos_[i] = pos; // start from zero accumulated ticks
      }
      accum_pos_[i] += pos - last_pos_[i];
      last_pos_[i] = pos;
    }
    first_encoder_pass_ = false;

    // Convert accumulated ticks to radians. No negation for the right wheel:
    // the firmware already mirror-compensates odom_r (bldc.c uses "- up_or_down"
    // for the right counter), so both counters advance positive when driving
    // forward — matching the sign of the velocity states.
    hw_positions_[left_wheel] = 2.0 * M_PI * accum_pos_[left_wheel] / TICKS_PER_ROTATION;
    hw_positions_[right_wheel] = 2.0 * M_PI * accum_pos_[right_wheel] / TICKS_PER_ROTATION;
  }

} // namespace hoverboard_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    hoverboard_driver::hoverboard_driver, hardware_interface::SystemInterface)
