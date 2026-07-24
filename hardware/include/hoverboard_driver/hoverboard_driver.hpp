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

#ifndef HOVERBOARD_DRIVER_HPP_
#define HOVERBOARD_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "hoverboard_driver/config.hpp"
#include "hoverboard_driver/protocol.hpp"

namespace hoverboard_driver
{

  enum
  {
    /// @brief left wheel
    left_wheel,
    /// @brief right wheel
    right_wheel
  };

  /// @brief State shared between the ros2_control read/write thread and the
  /// helper node's executor thread. Parameters are atomics (executor writes,
  /// control loop reads); telemetry is a small mutex-guarded block (control
  /// loop writes on each valid frame, the 10 Hz publisher timer reads).
  struct SharedState
  {
    // Parameters
    std::atomic<bool> motors_enabled{false};
    std::atomic<double> auto_disable_timeout{120.0};
    std::atomic<bool> publish_debug{true};
    // Set on every motors_enabled=true parameter set (even same-value re-sets),
    // consumed by write(): "arm now" also after an auto-disable.
    std::atomic<bool> arm_edge{false};

    // Telemetry
    std::mutex mutex;
    double voltage{0.0};
    double temperature{0.0};
    double dc_curr[2]{0.0, 0.0};
    double iq_curr[2]{0.0, 0.0};
    double vel[2]{0.0, 0.0};
    double pos[2]{0.0, 0.0};
    double cmd[2]{0.0, 0.0};
    uint8_t motor_error[2]{0, 0};
    bool fw_motors_enabled{false};
    bool fw_serial_timeout{false};
    bool connected{false};
  };

  /// @brief Helper node: owns the dynamic parameters (motors_enabled,
  /// auto_disable_timeout, publish_debug) and publishes all topics from its own
  /// executor thread so the ros2_control loop does no DDS work.
  class hoverboard_driver_node : public rclcpp::Node
  {
  public:
    explicit hoverboard_driver_node(SharedState *state);

  private:
    void timerCallback();
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

    SharedState *state_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Debug topics (gated by publish_debug, 10 Hz)
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pos_pub_[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr curr_pub_[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr iq_pub_[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pub_;

    // Status topics (latched, published on change)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motors_enabled_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fw_timeout_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr error_pub_[2];

    // Last published values for the on-change topics
    bool first_status_pub_{true};
    bool last_connected_{false};
    bool last_fw_enabled_{false};
    bool last_fw_timeout_{false};
    uint8_t last_error_[2]{0, 0};

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  };

  /// @brief ros2_control hardware interface for the hoverboard mainboard
  /// (lepptu firmware fork, standby/flags protocol).
  class hoverboard_driver : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(hoverboard_driver);

    ~hoverboard_driver() override;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    void protocol_recv(const rclcpp::Time &time, uint8_t byte);
    void on_encoder_update(const rclcpp::Time &time, int16_t right_raw, int16_t left_raw);

    // Shared state must outlive the node/executor that reference it.
    SharedState shared_state_;
    std::shared_ptr<hoverboard_driver_node> hardware_publisher_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread spin_thread_;
    std::atomic<bool> spin_done_{false};

    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;

    double wheel_radius_{0.0};
    double max_velocity_radps_{0.0};
    std::string port_;
    int port_fd_{-1};

    // Serial frame reassembly
    unsigned int msg_len_{0};
    uint8_t prev_byte_{0};
    uint8_t *p_{nullptr};
    SerialFeedback msg_;

    // Link state (control thread only)
    rclcpp::Time last_valid_frame_;
    bool have_valid_frame_{false};
    bool fw_enabled_rt_{false};
    bool fw_connected_rt_{false};

    // Odometry state (control thread only; reset in on_activate)
    int low_wrap_{0};
    int high_wrap_{0};
    int16_t last_wheelcount_[2]{0, 0}; // raw firmware counts [0, ENCODER_MAX)
    int mult_[2]{0, 0};                // completed wraps
    double last_pos_[2]{0.0, 0.0};     // unwrapped ticks at previous frame
    double accum_pos_[2]{0.0, 0.0};    // accumulated ticks since activation (board-restart proof)
    bool first_encoder_pass_{true};

    // Arming / auto-disable (control thread only)
    bool arm_request_{false};
    bool prev_allowed_{false};
    rclcpp::Time last_cmd_activity_;
    bool have_cmd_activity_{false};

    rclcpp::Clock steady_clock_{RCL_STEADY_TIME}; // for throttled logging
  };

} // namespace hoverboard_driver

#endif // HOVERBOARD_DRIVER_HPP_
