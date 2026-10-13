// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.

#include "litearm_vr_teleop/vr_forward_controller.hpp"

#include <chrono>
#include <exception>
#include <string>
#include <vector>

#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace litearm_vr_teleop
{
namespace
{

/// Floor on the interval a step is spread over, in seconds.
///
/// The velocity written to the hardware is the step divided by the interval
/// between the two commands it sits between, so a burst of commands arriving in
/// the same millisecond would otherwise ask for an unbounded velocity. 5 ms is
/// an order of magnitude below the interval a real command stream produces and
/// still two orders above the resolution of the receipt clock.
constexpr double kMinCommandInterval = 0.005;

/// Monotonic seconds. Used for the staleness test instead of the controller
/// manager's clock, which is subject to time jumps (the manager is usually
/// started together with a simulation or a bag replay, where /clock can step).
double monotonicSeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// Read a parameter, declaring it only if it is not there yet.
///
/// A bare declare_parameter is not safe here: recent ros2_control versions
/// auto declare every parameter the YAML places under this controller's node,
/// and declaring one twice fails the whole controller load with a message that
/// reads like a plugin registration problem. Same guard as the arm's servo
/// controller, for the same reason.
template <typename T>
T readParameter(
  rclcpp_lifecycle::LifecycleNode & node, const std::string & name, const T & fallback)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, fallback);
}

}  // namespace

void VrForwardController::declare_parameters()
{
  // The base class calls this from its own on_init. Declaring here rather than
  // reading here keeps the two stages separate: this one says what the
  // controller understands, read_parameters() decides what it will use.
  auto & node = *get_node();
  (void)readParameter<std::vector<std::string>>(
    node, "joints", std::vector<std::string>{});
  (void)readParameter<double>(node, "max_velocity", 1.0);
  (void)readParameter<double>(node, "command_timeout_s", 0.2);
  (void)readParameter<std::string>(node, "command_topic", "~/commands");
}

controller_interface::CallbackReturn VrForwardController::read_parameters()
{
  const auto logger = get_node()->get_logger();
  // A controller whose node cannot see its own parameters reports an empty
  // parameter, which reads like a configuration mistake in the controller. It is
  // usually a naming collision instead: a controller node remapped onto the
  // controller manager's name sees the manager's parameters, not its own. Naming
  // the node in the log makes that distinguishable at a glance.
  RCLCPP_DEBUG(
    logger, "reading parameters of controller node '%s'", get_node()->get_name());
  try {
    joint_names_ = readParameter<std::vector<std::string>>(
      *get_node(), "joints", std::vector<std::string>{});
    max_velocity_ = readParameter<double>(*get_node(), "max_velocity", 1.0);
    command_timeout_s_ = readParameter<double>(*get_node(), "command_timeout_s", 0.2);
    command_topic_ = readParameter<std::string>(*get_node(), "command_topic", "~/commands");
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "reading parameters failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (joint_names_.empty()) {
    RCLCPP_ERROR(
      logger,
      "parameter 'joints' is empty; the order of this list IS the order of the values "
      "on the command topic, so it cannot be left to a default");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (max_velocity_ <= 0.0) {
    RCLCPP_WARN(
      logger,
      "max_velocity=%.3f disables the velocity ceiling. On this arm the velocity field is "
      "the slew ceiling of the position reference, so an unbounded value means the firmware "
      "chooses the speed; expect a fault rather than a smooth move",
      max_velocity_);
  }
  if (command_timeout_s_ <= 0.0) {
    RCLCPP_WARN(
      logger,
      "command_timeout_s=%.3f disables the staleness test: if the publisher dies the "
      "controller keeps writing the last setpoint instead of holding",
      command_timeout_s_);
  }

  // The base class turns this list into the interface grid, one group of
  // interfaces per joint. Order within the group follows this list, which is why
  // position is first here and why the interface resolution in on_activate looks
  // the interfaces up by name instead of by index.
  interface_name_ = hardware_interface::HW_IF_POSITION;
  command_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
  };

  const std::string velocity_note = max_velocity_ > 0.0 ?
    (std::to_string(max_velocity_) + " rad/s") : std::string("disabled");
  RCLCPP_INFO(
    logger,
    "forward controller configured for %zu joints, command topic '%s', velocity ceiling %s, "
    "staleness %.3f s",
    joint_names_.size(), command_topic_.c_str(), velocity_note.c_str(), command_timeout_s_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VrForwardController::on_init()
{
  const auto base_return = forward_command_controller::ForwardControllersBase::on_init();
  if (base_return != controller_interface::CallbackReturn::SUCCESS) {
    return base_return;
  }
  // Read the joints here as well, not only in on_configure: the interface
  // configuration is queried before configure in some controller manager
  // versions, and a controller that claims no interfaces fails to activate with
  // a message about missing hardware rather than about a missing parameter.
  return read_parameters();
}

controller_interface::InterfaceConfiguration
VrForwardController::command_interface_configuration() const
{
  // Overridden rather than inherited, and the reason is worth recording because
  // the failure it prevents is not a warning.
  //
  // The base class is supposed to compose `<joint>/<interface>` names from its
  // joint list and its interface list, and the arm's stock forward controllers do
  // rely on that. On the ros2_control installed here (2.54.2) it returns the bare
  // interface names instead - `["position", "velocity"]` - and the controller
  // manager does not treat that as an empty configuration, it treats it as a
  // malformed interface name and ABORTS THE WHOLE MANAGER with
  //   Character '/', was not find in the interface name 'position'.
  // which takes down every controller in the process, not just this one.
  //
  // Composing the names here costs two lines and removes the dependency on which
  // side of that convention a given ros2_control version implements.
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(joint_names_.size() * 2u);
  for (const auto & joint : joint_names_) {
    // Both interfaces, always, and position first: the firmware reads the
    // velocity field as the slew ceiling of the position reference, so a
    // controller that claims position alone freezes the joint.
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
VrForwardController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(joint_names_.size());
  for (const auto & joint : joint_names_) {
    // Position is read for one reason: to seed the tracker at the measured pose
    // when the controller activates. Without it the first command would be
    // measured against a default, and the velocity derived from that distance
    // would be a number nobody asked for.
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return configuration;
}

controller_interface::CallbackReturn VrForwardController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto logger = get_node()->get_logger();

  // deliberately not calling the base's on_configure: it creates the same
  // subscription without recording arrival times, and two subscriptions on one
  // topic inside one node would both fire.
  if (joint_names_.empty()) {
    RCLCPP_ERROR(logger, "on_configure ran before the joints were read; nothing to control");
    return controller_interface::CallbackReturn::ERROR;
  }

  rt_command_ptr_.writeFromNonRT(nullptr);
  command_receipt_time_.writeFromNonRT(-1.0);

  try {
    command_subscription_ = get_node()->create_subscription<CmdType>(
      command_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const CmdType::SharedPtr message) {on_command(message);});
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      logger, "could not subscribe to the command topic '%s': %s", command_topic_.c_str(),
      error.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  tracker_ = std::make_unique<ForwardCommandTracker>(
    ForwardCommandLimits{joint_names_.size(), max_velocity_, command_timeout_s_});

  RCLCPP_INFO(
    logger, "subscribed to '%s' (QoS SystemDefaults, matching the teleop node's publisher)",
    command_topic_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

void VrForwardController::on_command(const CmdType::SharedPtr message)
{
  // Two independent facts go to the control loop: the values, and when they
  // arrived. They are written in that order so a cycle that sees the new values
  // also sees the matching timestamp.
  rt_command_ptr_.writeFromNonRT(message);
  command_receipt_time_.writeFromNonRT(monotonicSeconds());
}

controller_interface::CallbackReturn VrForwardController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto logger = get_node()->get_logger();
  if (tracker_ == nullptr) {
    RCLCPP_ERROR(logger, "on_activate ran before on_configure");
    return controller_interface::CallbackReturn::ERROR;
  }

  const bool resolved =
    controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION, position_commands_) &&
    controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names_, hardware_interface::HW_IF_VELOCITY, velocity_commands_) &&
    controller_interface::get_ordered_interfaces(
      state_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION, position_states_);
  if (!resolved) {
    RCLCPP_ERROR(
      logger,
      "not enough interfaces: every joint needs a position and a velocity command interface "
      "and a position state interface. Velocity is not optional here (a position only writer "
      "leaves the reference frozen on this arm). Check the interfaces exported by the "
      "<ros2_control> block of the description against this controller's 'joints'");
    return controller_interface::CallbackReturn::ERROR;
  }

  const Eigen::Index count = static_cast<Eigen::Index>(joint_names_.size());
  measured_ = Eigen::VectorXd::Zero(count);
  target_ = Eigen::VectorXd::Zero(count);
  target_velocity_ = Eigen::VectorXd::Zero(count);
  command_positions_ = Eigen::VectorXd::Zero(count);
  command_velocities_ = Eigen::VectorXd::Zero(count);
  has_target_ = false;
  noted_derived_velocity_ = false;

  for (Eigen::Index i = 0; i < count; ++i) {
    measured_(i) = position_states_[static_cast<std::size_t>(i)].get().get_value();
  }
  tracker_->reset(measured_);

  // Drop any command that arrived before activation: it was aimed at a robot
  // that was not being controlled, and acting on it now would be a jump.
  rt_command_ptr_.writeFromNonRT(nullptr);
  command_receipt_time_.writeFromNonRT(-1.0);

  RCLCPP_INFO(
    logger,
    "forward controller activated, holding at the measured pose until the first command");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VrForwardController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  position_commands_.clear();
  velocity_commands_.clear();
  position_states_.clear();
  has_target_ = false;
  if (tracker_ != nullptr && measured_.size() > 0) {
    // The next activation must seed from a fresh measurement, not from the pose
    // this controller was holding when it was told to let go.
    tracker_->reset(measured_);
  }
  RCLCPP_INFO(get_node()->get_logger(), "forward controller deactivated, interfaces released");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type VrForwardController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const std::size_t count = joint_names_.size();
  if (
    tracker_ == nullptr || position_commands_.size() != count ||
    velocity_commands_.size() != count || position_states_.size() != count)
  {
    return controller_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < count; ++i) {
    measured_(static_cast<Eigen::Index>(i)) = position_states_[i].get().get_value();
  }

  const auto rt_command = rt_command_ptr_.readFromRT();
  const CmdType * incoming = (rt_command != nullptr && *rt_command) ? rt_command->get() : nullptr;
  bool has_stated_velocity = false;
  if (incoming != nullptr) {
    // Two layouts are accepted, and the message itself says which one it is:
    // one value per joint, or a position and the velocity to carry it for every
    // joint. The second is what this stack's own node publishes, because it runs
    // the rate limiter and therefore knows the speed it is asking for; a
    // publisher that has only positions to give still works, and the velocity is
    // then derived from its stream.
    has_stated_velocity = incoming->data.size() == 2 * count;
    if (incoming->data.size() != count && !has_stated_velocity) {
      // A wrong sized command is a publisher bug. Holding and saying so once per
      // few seconds is better than writing whatever happens to fit, which would
      // move the wrong joint or leave part of the arm behind.
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "command has %zu values but %zu joints are configured; expected %zu (positions) or %zu "
        "(positions then velocities). Ignoring it and holding",
        incoming->data.size(), count, count, 2 * count);
      incoming = nullptr;
      // The cached setpoint from the last good command is still the one written,
      // so the velocity that belongs to it is the derived one: the stated field
      // was meant for values this cycle is refusing.
      has_stated_velocity = false;
    }
  }

  if (incoming != nullptr) {
    for (std::size_t i = 0; i < count; ++i) {
      target_(static_cast<Eigen::Index>(i)) = incoming->data[i];
    }
    if (has_stated_velocity) {
      for (std::size_t i = 0; i < count; ++i) {
        target_velocity_(static_cast<Eigen::Index>(i)) = incoming->data[count + i];
      }
    } else if (!noted_derived_velocity_) {
      // Said once per activation, not per cycle: a position only publisher works
      // and is not an error, but it is worth knowing why the ceiling moves on
      // its own - this controller is then differentiating the stream, and that
      // derivation carries the arrival interval measured on this loop's clock.
      RCLCPP_INFO(
        get_node()->get_logger(),
        "commands carry positions only; the slew ceiling is being derived from the interval "
        "between arrivals. Publishing %zu values (positions then velocities) states it instead",
        2 * count);
      noted_derived_velocity_ = true;
    }
    has_target_ = true;
  }

  const double * receipt = command_receipt_time_.readFromRT();
  const double command_age = (receipt != nullptr && *receipt >= 0.0) ?
    (monotonicSeconds() - *receipt) : -1.0;

  // The interval the step from the previous setpoint to this one happened over.
  // Only a position only publisher needs it, because it is only then that the
  // velocity has to be derived: the firmware reads that field as the slew
  // ceiling of the position reference, so the honest derived value is the step
  // over the time the sender took to make it. Using the control period instead
  // inflates the request whenever the command stream is slower than the
  // controller. At 23 Hz a 0.02 rad step became 2.0 rad/s instead of 0.46, which
  // the ceiling then clipped to 1.0 - every step asked for full speed. The
  // interval between arrivals is measured from the receipt times, and floored so
  // that a burst of commands cannot ask for an unbounded velocity. When the
  // publisher states the velocity, this figure is not used at all.
  double command_interval = period.seconds();
  bool new_command = false;
  if (receipt != nullptr && *receipt >= 0.0 && *receipt != last_command_receipt_) {
    if (last_command_receipt_ >= 0.0) {
      command_interval = std::max(kMinCommandInterval, *receipt - last_command_receipt_);
    }
    last_command_receipt_ = *receipt;
    // Only this cycle has a step to differentiate. See ForwardCommandTracker:
    // recomputing the velocity on the cycles in between puts zero in the field,
    // and a zero slew ceiling half the time is an arm that moves half as far as
    // it was told to.
    new_command = true;
  }

  // No command has ever arrived: hold where the arm is. `reset()` at activation
  // already put the tracker at the measured pose, so passing a null target here
  // is a hold, not a move to zero.
  tracker_->update(
    has_target_ ? &target_ : nullptr, has_stated_velocity ? &target_velocity_ : nullptr,
    measured_, command_age, new_command, command_interval,
    command_positions_, command_velocities_);

  for (std::size_t i = 0; i < count; ++i) {
    position_commands_[i].get().set_value(command_positions_(static_cast<Eigen::Index>(i)));
    velocity_commands_[i].get().set_value(command_velocities_(static_cast<Eigen::Index>(i)));
  }
  return controller_interface::return_type::OK;
}

}  // namespace litearm_vr_teleop

PLUGINLIB_EXPORT_CLASS(
  litearm_vr_teleop::VrForwardController, controller_interface::ControllerInterface)
