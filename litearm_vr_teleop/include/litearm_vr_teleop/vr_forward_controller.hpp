// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.

#ifndef LITEARM_VR_TELEOP__VR_FORWARD_CONTROLLER_HPP_
#define LITEARM_VR_TELEOP__VR_FORWARD_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <controller_interface/controller_interface.hpp>
#include <forward_command_controller/forward_controllers_base.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "litearm_vr_teleop/forward_command.hpp"

namespace litearm_vr_teleop
{

/// Forward controller for teleoperated position setpoints.
///
/// It derives from `forward_command_controller::ForwardControllersBase` because
/// the governing idea is the forward controller's: a stream of setpoints arrives
/// on a topic and is passed straight to the hardware interfaces, with no
/// trajectory, no interpolation and no notion of a goal. What it adds is the one
/// thing this arm cannot do without.
///
/// The base class maps message element i to command interface i. That is correct
/// for a controller that writes one interface per joint, and it is why the stock
/// `ForwardCommandController` cannot drive this arm: the firmware reads the
/// velocity field as the slew ceiling of the position reference
/// (`litearm_ros2_control` exports the interface, the daemon forwards it, and
/// the firmware freezes the reference when it is zero). Writing position alone
/// therefore produces an arm that accepts commands and does not move.
///
/// Two ways out were available. One is to derive from the multi interface
/// variant and require the publisher to send fourteen numbers whose ordering
/// must match the interface ordering the base produces. The other, taken here,
/// is to override `update()` and derive each joint's velocity from the setpoint
/// stream, so the topic keeps the shape its publisher naturally has: seven
/// positions, one per joint, in joint order.
///
/// The topic accepts both layouts and the message itself says which one it is,
/// because the two publishers are not equally informed. This stack's own node
/// runs the rate limiter before it publishes, so it knows the speed it is asking
/// for and publishes fourteen values: seven positions, then the seven
/// velocities to carry them. A publisher that has only positions to give still
/// works, and the velocity is then derived from its stream as described above -
/// with the one difference that the derived figure carries the interval between
/// the controller's arrivals, measured on the controller's clock, which is not
/// the sender's clock.
///
/// What is still inherited and worth naming: `rt_command_ptr_`, the lock free
/// handover from the subscription to the control loop, and the joint list
/// plumbing. What is overridden and why:
///
///   `command_interface_configuration`  composes the interface names. The base's
///     version returns the bare interface names on the ros2_control installed
///     here, which the manager rejects as malformed and answers by aborting the
///     whole manager process.
///   `state_interface_configuration`  claims position, which is what the tracker
///     is seeded from at activation.
///   `on_init`  reads the joint list early so the configurations above know it.
///   `on_configure`  creates the command subscription itself, so that each
///     message's arrival time can be recorded. Staleness cannot be judged from
///     the value alone, because a hold is a legitimate value.
///   `on_activate` / `on_deactivate` / `update`  because the base's versions
///     assume one value per interface, and this controller maps one value to two.
class VrForwardController : public forward_command_controller::ForwardControllersBase
{
public:
  /// The message the base class's subscription is typed for, under a name this
  /// class can use: the base's alias lives in its own namespace and is not
  /// inherited.
  using CmdType = std_msgs::msg::Float64MultiArray;

  VrForwardController() = default;
  ~VrForwardController() override = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  void declare_parameters() override;
  controller_interface::CallbackReturn read_parameters() override;

private:
  /// Subscription callback. Runs on an executor thread, never in the control
  /// loop, so it may read a clock but must not allocate its way into the loop:
  /// the only things it touches are the two realtime buffers.
  void on_command(const CmdType::SharedPtr message);

  std::string command_topic_;
  double max_velocity_ = 1.0;
  double command_timeout_s_ = 0.2;

  rclcpp::Subscription<CmdType>::SharedPtr command_subscription_;
  /// Arrival time of the most recent command, on a monotonic clock.
  /// A negative value means "no command yet".
  realtime_tools::RealtimeBuffer<double> command_receipt_time_;
  /// Arrival time of the last command this loop consumed, used to measure the
  /// interval a step is spread over. Not realtime shared: only update() touches
  /// it. -1 until the first command arrives.
  double last_command_receipt_ = -1.0;

  std::unique_ptr<ForwardCommandTracker> tracker_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  position_commands_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  velocity_commands_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  position_states_;

  /// Scratch buffers, sized once at activation so the control loop never
  /// allocates.
  Eigen::VectorXd measured_;
  Eigen::VectorXd target_;
  /// The velocity the publisher stated alongside the positions, in the second
  /// half of a command that carries one. Unused when the command has only
  /// positions in it.
  Eigen::VectorXd target_velocity_;
  Eigen::VectorXd command_positions_;
  Eigen::VectorXd command_velocities_;
  bool has_target_ = false;
  /// Whether this activation has already said that the ceiling is being derived
  /// rather than stated. Logged once, not once per cycle.
  bool noted_derived_velocity_ = false;
};

}  // namespace litearm_vr_teleop

#endif  // LITEARM_VR_TELEOP__VR_FORWARD_CONTROLLER_HPP_
