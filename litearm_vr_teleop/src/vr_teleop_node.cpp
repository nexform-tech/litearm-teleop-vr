// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.
//
// The teleoperation node: headset frames in, forward controller commands out.
//
// Where the work is split
// -----------------------
//   headset_link.py      transport. Reads the WebSocket and publishes raw frames.
//   this node            semantics, planning, and the command stream.
//   VrForwardController  the hardware path: position plus the velocity that makes
//                        the position reachable on this arm.
//
// The node runs a fixed rate control loop rather than reacting to each headset
// frame. Frames arrive when the headset feels like it (about 72 Hz, with jitter);
// the controller and the firmware want a steady stream at a rate the node
// chooses. Between two frames the loop keeps solving against the SAME absolute
// Cartesian target, which is idempotent: the target does not depend on how many
// cycles have passed since it was computed, so re-solving cannot accumulate
// motion. That property is what makes a fixed rate loop safe here, and it is why
// the increment mapping takes the measured end effector pose as an input instead
// of keeping a running target of its own.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "litearm_vr_teleop/joint_move.hpp"
#include "litearm_vr_teleop/msg/vr_controller_frame.hpp"
#include "litearm_vr_teleop/teleop_math.hpp"
#include "litearm_vr_teleop/teleop_planner.hpp"
#include "litearm_vr_teleop/teleop_start_pose.hpp"

namespace litearm_vr_teleop
{
namespace
{

/// Speed and acceleration of the move to the start pose, in rad/s and rad/s^2.
///
/// A quarter of the arm's rated joint speed. Returning to the start pose is the
/// one motion that happens without the operator holding the grip, which makes it
/// the one motion nobody is watching; it is deliberately slow.
constexpr double kStartPoseSpeed = 0.5;
constexpr double kStartPoseAccel = 3.0;

/// Monotonic seconds, for the debounce and watchdog windows. A world clock would
/// make a simulated setup with a stepping /clock open the gripper at random.
double steadySeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

class VrTeleopNode : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GripperCommandGoalHandle = rclcpp_action::Client<GripperCommand>::GoalHandle;

  /// What the node is doing. The transitions are where the safety lives, so they
  /// are all in one place (`tick`).
  enum class Phase
  {
    /// No joint state has arrived yet, so there is nothing to plan from.
    kWaitingForState,
    /// Connected and standing by. The arm holds wherever it is.
    kIdle,
    /// Moving to the teleoperation start pose at equal joint speed.
    kReturningToStart,
    /// Following the headset.
    kFollowing,
    /// Stopped by request. Nothing moves until the start service is called.
    kStopped,
  };

  explicit VrTeleopNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("vr_teleop", options)
  {
    declareParameters();
  }

  /// Build everything that needs a shared_ptr to this node.
  ///
  /// Separate from the constructor because the planner hands the node to MoveIt's
  /// model loader and to the kinematics plugin, and doing that in the constructor
  /// would mean handing out a half-built object.
  void initialize()
  {
    mapper_ = std::make_unique<TeleopIncrementMapper>(buildAxisMapping(), teleop_limits_);
    filter_ = std::make_unique<JointTargetFilter>(teleop_limits_, joint_names_.size());

    // The mapping is settable while the node runs, because the only person who
    // can tell whether a sign is right is the one holding the controller, and
    // making them restart the stack to try one is how a five second decision
    // turns into an afternoon. The callback keeps the mapper's anchor: the arm
    // does not jump when a sign changes.
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return onParameterUpdate(parameters);
      });

    planner_error_.clear();
    planner_ = std::make_unique<TeleopPlanner>(planner_options_, shared_from_this());
    if (!planner_->initialize(planner_error_)) {
      // Not fatal. The node still runs, still holds, and still reports why it
      // cannot follow: a teleoperation node that exits on a bad configuration
      // leaves the operator unable to tell a software fault from a hardware one.
      RCLCPP_ERROR(get_logger(), "planning is unavailable: %s", planner_error_.c_str());
      planner_ready_ = false;
    } else {
      planner_ready_ = true;
    }

    // The controller subscribes with SystemDefaultsQoS; publishing with anything
    // else is a silent incompatibility, so the QoS is stated rather than
    // defaulted.
    command_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      command_topic_, rclcpp::SystemDefaultsQoS());

    frame_subscription_ = create_subscription<msg::VrControllerFrame>(
      frame_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const msg::VrControllerFrame::SharedPtr frame) {onFrame(frame);});

    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr state) {onJointState(state);});

    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {onStop(response);});
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {onStart(response);});

    if (gripper_enabled_) {
      gripper_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_action_);
    }

    const double rate = std::max(1.0, control_rate_hz_);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {tick();});

    RCLCPP_INFO(
      get_logger(),
      "VR teleop ready: frames on '%s', commands on '%s' at %.1f Hz, planning backend '%s'",
      frame_topic_.c_str(), command_topic_.c_str(), rate,
      planner_ready_ ? planner_->backendName().c_str() : "unavailable");
    RCLCPP_INFO(
      get_logger(),
      "start pose (grasp_ready) [%.6f %.6f %.6f %.6f %.6f %.6f %.6f], returned to at %.2f rad/s",
      start_pose_(0), start_pose_(1), start_pose_(2), start_pose_(3), start_pose_(4),
      start_pose_(5), start_pose_(6), start_pose_speed_);
    if (gripper_enabled_) {
      RCLCPP_INFO(
        get_logger(), "gripper commands go to '%s' (open %.5f m, close %.5f m)",
        gripper_action_.c_str(), gripper_open_position_, gripper_close_position_);
    }
    RCLCPP_INFO(
      get_logger(),
      "press the grip to follow (the arm first returns to the start pose), release it to hold, "
      "pull the trigger to close the gripper");
    RCLCPP_INFO(
      get_logger(),
      "controller rotation is read in the %s frame, signs [%.2f %.2f %.2f]; "
      "'ros2 param set /vr_teleop rotation_mode tool' and 'ros2 param set /vr_teleop "
      "rotation_signs [a, b, c]' change it while this node runs",
      rotation_mode_name_.c_str(), rotation_signs_.x(), rotation_signs_.y(), rotation_signs_.z());
    RCLCPP_INFO(
      get_logger(),
      "hand increments are measured against the %s pose, target margin %.0f mm / %.2f mrad, "
      "lead capped at %.3f m / %.3f rad",
      reference_from_command_ ? "commanded" : "measured", target_deadband_m_ * 1000.0,
      target_deadband_rad_ * 1000.0, max_lead_m_, max_lead_rad_);
  }

private:
  /// Read a parameter, declaring it only if it is not there yet.
  ///
  /// This node is created with `automatically_declare_parameters_from_overrides`,
  /// because the kinematics solver it loads expects to find its own tuning as
  /// ordinary parameters (`litearm_arm.solver_type` and friends) and reads them
  /// with `has_parameter` / `get_parameter`. That flag declares the overrides
  /// before the constructor runs, so a bare `declare_parameter` here would throw
  /// "already declared" for every parameter the launch file passes, which is most
  /// of them. The guarded read is the same pattern the arm's servo controller
  /// uses, for the same reason.
  template <typename T>
  T readParameter(const std::string & name, const T & fallback)
  {
    if (has_parameter(name)) {
      return get_parameter(name).get_value<T>();
    }
    return declare_parameter<T>(name, fallback);
  }

  void declareParameters()
  {
    frame_topic_ = readParameter<std::string>("frame_topic", "/vr_controller_frame");
    joint_state_topic_ = readParameter<std::string>("joint_state_topic", "/joint_states");
    command_topic_ =
      readParameter<std::string>("command_topic", "/vr_forward_controller/commands");
    control_rate_hz_ = readParameter<double>("control_rate_hz", 100.0);

    joint_names_ = readParameter<std::vector<std::string>>(
      "joint_names",
      std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
        "joint7"});

    std::vector<double> start_pose(std::begin(kTeleopStartPose), std::end(kTeleopStartPose));
    start_pose = readParameter<std::vector<double>>("start_pose", start_pose);
    start_pose_speed_ = readParameter<double>("start_pose_speed", kStartPoseSpeed);
    start_pose_accel_ = readParameter<double>("start_pose_accel", kStartPoseAccel);

    // The point of the tool that follows the hand, as an offset from tip_link in
    // the tip frame (config/tcp.yaml). Defaults to the current gripper's
    // flange-to-grasp-point distance.
    const std::vector<double> tcp_offset =
      readParameter<std::vector<double>>("tcp_offset", std::vector<double>{0.0, 0.0, 0.18});
    if (tcp_offset.size() != 3u) {
      RCLCPP_WARN(
        get_logger(), "tcp_offset needs three numbers, got %zu; teleoperating the tip link",
        tcp_offset.size());
      tcp_offset_ = Eigen::Vector3d::Zero();
    } else {
      tcp_offset_ = Eigen::Vector3d(tcp_offset[0], tcp_offset[1], tcp_offset[2]);
    }

    TeleopPlanner::Options planner_options;
    const std::string backend = readParameter<std::string>("ik_backend", "plugin");
    if (backend == "dls") {
      planner_options.backend = TeleopPlanner::IkBackend::kDls;
    } else if (backend != "plugin") {
      RCLCPP_WARN(
        get_logger(), "ik_backend '%s' is not a backend; using 'plugin'", backend.c_str());
    }
    planner_options.group_name = readParameter<std::string>("group_name", "litearm_arm");
    planner_options.base_frame = readParameter<std::string>("base_frame", "base_link");
    planner_options.tip_link = readParameter<std::string>("tip_link", "ee_link");
    planner_options.position_tolerance = readParameter<double>("ik_position_tolerance", 3e-3);
    planner_options.rotation_tolerance = readParameter<double>("ik_rotation_tolerance", 0.05);
    planner_options.dls_damping = readParameter<double>("dls_damping", 0.05);
    planner_options.dls_null_space_gain = readParameter<double>("dls_null_space_gain", 0.2);
    planner_options.dls_max_iterations = readParameter<int>("dls_max_iterations", 120);
    planner_options.dls_max_step_rad = readParameter<double>("dls_max_step_rad", 0.2);
    planner_options.singularity_warning_threshold =
      readParameter<double>("singularity_warning_threshold", 0.02);
    planner_options_ = planner_options;

    teleop_limits_.max_step_rad = readParameter<double>("max_step_rad", 0.08);
    teleop_limits_.max_joint_velocity = readParameter<double>("max_joint_velocity", 2.0);
    teleop_limits_.low_pass_tau = readParameter<double>("low_pass_tau", 0.02);
    teleop_limits_.moving_average_frames = static_cast<std::size_t>(
      std::max(1, readParameter<int>("moving_average_frames", 10)));

    // How the controller's rotation reaches the tool. `world` is the geometric
    // default: the tool turns about the same world axis as the controller,
    // whatever pose the arm is in. `tool` is the previous behaviour, kept
    // because a rig that was calibrated for it can ask for it back.
    rotation_mode_name_ = readParameter<std::string>("rotation_mode", "world");
    if (rotation_mode_name_ != "world" && rotation_mode_name_ != "tool") {
      RCLCPP_WARN(
        get_logger(), "rotation_mode '%s' is not a mode; using 'world'",
        rotation_mode_name_.c_str());
      rotation_mode_name_ = "world";
    }
    const std::vector<double> default_signs{1.0, 1.0, 1.0};
    const std::vector<double> rotation_signs =
      readParameter<std::vector<double>>("rotation_signs", default_signs);
    if (rotation_signs.size() != 3u) {
      RCLCPP_WARN(
        get_logger(), "rotation_signs needs three numbers, got %zu; using [1 1 1]",
        rotation_signs.size());
      rotation_signs_ = Eigen::Vector3d::Ones();
    } else {
      rotation_signs_ = Eigen::Vector3d(rotation_signs[0], rotation_signs[1], rotation_signs[2]);
    }

    // What the increment mapping measures against, and how far the target may
    // run ahead of the arm. See config/vr_teleop.yaml for what each mode is for;
    // `commanded` is the one that restores the tracking ratio on a plant that
    // lags, and it is off by default because it is a decision to make after
    // measuring on the hardware rather than before.
    const std::string reference_pose = readParameter<std::string>("reference_pose", "measured");
    if (reference_pose != "measured" && reference_pose != "commanded") {
      RCLCPP_WARN(
        get_logger(), "reference_pose '%s' is not a source; using 'measured'",
        reference_pose.c_str());
      reference_from_command_ = false;
    } else {
      reference_from_command_ = reference_pose == "commanded";
    }
    target_deadband_m_ = readParameter<double>("target_deadband_m", 0.0);
    target_deadband_rad_ = readParameter<double>("target_deadband_rad", 0.0);
    max_lead_m_ = readParameter<double>("max_target_lead_m", 0.15);
    max_lead_rad_ = readParameter<double>("max_target_lead_rad", 0.6);

    headset_timeout_s_ = readParameter<double>("headset_timeout_s", 0.5);

    gripper_enabled_ = readParameter<bool>("gripper_enabled", true);
    gripper_action_ =
      readParameter<std::string>("gripper_action", "/gripper_controller/gripper_cmd");
    gripper_open_position_ = readParameter<double>("gripper_open_position", 0.07479);
    gripper_close_position_ = readParameter<double>("gripper_close_position", 0.0);
    // Not a torque command. The gripper's own controller allocates torque from a
    // budget that is a deployment parameter; this value is the one its stall
    // detection compares against, so it has to stay in the same order of
    // magnitude as the gripper's rated torque.
    gripper_max_effort_ = readParameter<double>("gripper_max_effort", 3.5);
    trigger_threshold_ = readParameter<double>("trigger_threshold", 0.5);
    trigger_debounce_s_ = readParameter<double>("trigger_debounce_s", 0.05);
    // A grip that closes onto an object never reaches the commanded position, and
    // operators release the trigger while the fingers are still moving. Opening
    // immediately would leave them with no grasp at all, so a release inside this
    // window is ignored and the open follows once the window has passed.
    min_close_s_ = readParameter<double>("min_close_s", 0.5);

    if (start_pose.size() != joint_names_.size()) {
      RCLCPP_ERROR(
        get_logger(),
        "start_pose has %zu entries but %zu joints are configured; using the built-in "
        "grasp_ready pose instead",
        start_pose.size(), joint_names_.size());
      start_pose.assign(std::begin(kTeleopStartPose), std::end(kTeleopStartPose));
    }
    start_pose_ = Eigen::VectorXd::Map(start_pose.data(), start_pose.size());
  }

  /// The mapping the parameters describe, on top of the measured baseline.
  ///
  /// The baseline carries the frame correspondence, which is a property of where
  /// the operator stands; these two parameters choose how the controller's
  /// rotation is read and, if the rig needs it, flip a rotation axis. They
  /// override rather than extend the baseline because a mapping is one thing:
  /// half of one and half of another is a third mapping nobody chose.
  AxisMapping buildAxisMapping() const
  {
    AxisMapping mapping = singleArmUprightMapping();
    mapping.rotation_mode =
      rotation_mode_name_ == "tool" ? RotationMapping::kTool : RotationMapping::kWorld;
    mapping.rotation_signs = rotation_signs_;
    return mapping;
  }

  /// Apply a run time change to the mapping.
  ///
  /// Rejects the change rather than correcting it: an operator who typed three
  /// numbers expects those three, and a silently repaired fourth value is a
  /// mapping that does not match the one they think they set.
  rcl_interfaces::msg::SetParametersResult onParameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::string mode = rotation_mode_name_;
    Eigen::Vector3d signs = rotation_signs_;
    bool mapping_changed = false;

    for (const rclcpp::Parameter & parameter : parameters) {
      if (parameter.get_name() == "rotation_mode") {
        const std::string requested = parameter.as_string();
        if (requested != "world" && requested != "tool") {
          result.successful = false;
          result.reason = "rotation_mode must be 'world' or 'tool', not '" + requested + "'";
          return result;
        }
        mode = requested;
        mapping_changed = true;
      } else if (parameter.get_name() == "rotation_signs") {
        const std::vector<double> requested = parameter.as_double_array();
        if (requested.size() != 3u) {
          result.successful = false;
          result.reason = "rotation_signs needs three numbers";
          return result;
        }
        signs = Eigen::Vector3d(requested[0], requested[1], requested[2]);
        mapping_changed = true;
      }
    }

    if (!mapping_changed) {
      return result;
    }

    rotation_mode_name_ = mode;
    rotation_signs_ = signs;
    if (mapper_ != nullptr) {
      mapper_->setMapping(buildAxisMapping());
    }
    RCLCPP_WARN(
      get_logger(),
      "rotation mapping changed to mode '%s' signs [%.2f %.2f %.2f]; it applies from the next "
      "headset frame, and the arm holds its target until then",
      rotation_mode_name_.c_str(), rotation_signs_.x(), rotation_signs_.y(), rotation_signs_.z());
    return result;
  }

  // ---------------------------------------------------------------- callbacks

  void onFrame(const msg::VrControllerFrame::SharedPtr frame)
  {
    if (frame->side != "right") {
      // The single arm rig is driven by the right controller. Saying so once is
      // more useful than silently ignoring frames forever.
      RCLCPP_WARN_ONCE(
        get_logger(), "ignoring frames for side '%s'; this node follows the right controller",
        frame->side.c_str());
      return;
    }
    if (!frame->tracked) {
      // Tracking loss is not the same as the operator standing still, and the
      // values are stale. Accepting them would look identical to correct
      // behaviour until tracking resumes with a jump.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "the headset is not tracking the controller; ignoring frames");
      return;
    }

    latest_frame_ = *frame;
    last_frame_time_ = steadySeconds();
    latest_frame_valid_ = true;
    ++frame_sequence_;
  }

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr state)
  {
    if (state->position.size() != state->name.size()) {
      return;
    }
    Eigen::VectorXd positions =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const auto found = std::find(state->name.begin(), state->name.end(), joint_names_[i]);
      if (found == state->name.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "the joint state does not contain '%s'; the arm cannot be controlled without it",
          joint_names_[i].c_str());
        return;
      }
      positions(static_cast<Eigen::Index>(i)) =
        state->position[static_cast<std::size_t>(std::distance(state->name.begin(), found))];
    }
    measured_ = positions;
    have_measured_ = true;
  }

  void onStop(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    // Stopping is idempotent and always allowed. Everything else waits for an
    // explicit start.
    phase_ = Phase::kStopped;
    mapper_->reset();
    filter_->reset();
    start_move_.reset();
    clearFollowState();
    saw_grip_ = false;
    response->success = true;
    response->message = "stopped; the arm holds where it is until ~/start is called";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void onStart(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    if (!planner_ready_) {
      response->success = false;
      response->message = "planning is unavailable: " + planner_error_;
      return;
    }
    if (!have_measured_) {
      response->success = false;
      response->message = "no joint state has arrived yet";
      return;
    }
    phase_ = Phase::kIdle;
    response->success = true;
    response->message = "resumed; press the grip to follow";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  // ------------------------------------------------------------- control loop

  void tick()
  {
    const double now = steadySeconds();
    const double dt = now - last_tick_;
    last_tick_ = now;

    if (!have_measured_) {
      // Publishing here would mean publishing a pose nobody measured.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "waiting for %s", joint_state_topic_.c_str());
      return;
    }
    if (phase_ == Phase::kWaitingForState) {
      phase_ = Phase::kIdle;
    }

    if (phase_ != Phase::kStopped) {
      updateGripEdge();
      updateTriggerEdge(now);
      if (phase_ != Phase::kIdle) {
        applyHeadsetWatchdog();
      }
    }

    switch (phase_) {
      case Phase::kWaitingForState:
      case Phase::kIdle:
      case Phase::kStopped:
        publishHold();
        break;
      case Phase::kReturningToStart:
        stepReturnToStart(dt);
        break;
      case Phase::kFollowing:
        stepFollowing(dt);
        break;
    }
  }

  /// Detect the grip press and release.
  ///
  /// The edge is only taken from a frame the headset is currently tracking. A
  /// press that arrives while the link is dead must not be an event, because the
  /// next thing the node does is move the arm to the start pose.
  void updateGripEdge()
  {
    if (!latest_frame_valid_) {
      return;
    }
    const bool pressed = latest_frame_.grip_active;

    if (pressed && !saw_grip_) {
      saw_grip_ = true;
      if (phase_ == Phase::kStopped || !planner_ready_) {
        return;
      }
      beginReturnToStart();
      return;
    }

    if (!pressed && saw_grip_) {
      saw_grip_ = false;
      if (phase_ == Phase::kFollowing || phase_ == Phase::kReturningToStart) {
        mapper_->pause();
        filter_->reset();
        start_move_.reset();
        clearFollowState();
        phase_ = Phase::kIdle;
        RCLCPP_INFO(
          get_logger(), "grip released: holding. The next press returns to the start pose");
      }
    }
  }

  /// Turn the trigger analogue into two discrete gripper commands.
  ///
  /// Discrete rather than proportional because the gripper's controller is goal
  /// based: a grasp succeeds when the fingers stop against an object instead of
  /// reaching the commanded opening. Streaming a goal every frame would keep
  /// replacing the goal being judged, and the judgement is the useful part.
  void updateTriggerEdge(double now)
  {
    if (!latest_frame_valid_ || !gripper_enabled_ || gripper_client_ == nullptr) {
      return;
    }

    const bool pressed = latest_frame_.trigger > trigger_threshold_;
    if (pressed != trigger_raw_) {
      trigger_raw_ = pressed;
      trigger_changed_at_ = now;
    }
    if (pressed == trigger_stable_ || now - trigger_changed_at_ < trigger_debounce_s_) {
      return;
    }

    if (pressed) {
      trigger_stable_ = true;
      last_close_ = now;
      sendGripperGoal(gripper_close_position_, "close");
      return;
    }

    // Releasing inside the minimum close window would open the fingers before
    // they had a chance to hold anything.
    if (now - last_close_ < min_close_s_) {
      return;
    }
    trigger_stable_ = false;
    sendGripperGoal(gripper_open_position_, "open");
  }

  /// Freeze when the headset goes quiet.
  ///
  /// Without this, a headset that comes back at a completely different pose would
  /// be treated as continuous with the pose before it.
  void applyHeadsetWatchdog()
  {
    if (!latest_frame_valid_) {
      return;
    }
    const double quiet = steadySeconds() - last_frame_time_;
    if (quiet <= headset_timeout_s_) {
      return;
    }

    mapper_->reset();
    filter_->reset();
    start_move_.reset();
    clearFollowState();
    saw_grip_ = false;
    phase_ = Phase::kIdle;
    latest_frame_valid_ = false;
    RCLCPP_WARN(
      get_logger(), "no headset frame for %.2f s: holding, and the next grip press re-anchors",
      quiet);
  }

  void beginReturnToStart()
  {
    const Eigen::VectorXd speed =
      Eigen::VectorXd::Constant(measured_.size(), start_pose_speed_);
    const Eigen::VectorXd accel =
      Eigen::VectorXd::Constant(measured_.size(), start_pose_accel_);
    // The joint limits come from the robot model, so the start pose move is
    // bounded by the same range the planner works in. Without them a start pose
    // outside the range would be commanded in full and refused by the firmware,
    // which looks exactly like the move never starting.
    start_move_ = std::make_unique<JointMoveProfile>(
      measured_, start_pose_, speed, accel, planner_->lowerLimits(), planner_->upperLimits());
    filter_->reset();
    clearFollowState();
    phase_ = Phase::kReturningToStart;
    RCLCPP_INFO(
      get_logger(),
      "grip pressed: returning to the start pose at %.2f rad/s (all joints the same speed, so "
      "the joints with less to travel arrive first)", start_pose_speed_);
  }

  void stepReturnToStart(double dt)
  {
    if (start_move_ == nullptr) {
      phase_ = Phase::kIdle;
      return;
    }
    const JointMoveProfile::Point point = start_move_->step(dt);
    publishPositions(point.position);

    if (!point.done) {
      return;
    }

    // Anchoring happens here, at the end of the move, not at the press: the pose
    // that matters is the one the operator is holding when following begins.
    // Anchoring at the press would make the arm lurch by however much they moved
    // during the return.
    if (!latest_frame_valid_) {
      RCLCPP_WARN(
        get_logger(), "start pose reached but the headset went quiet; waiting for a fresh press");
      phase_ = Phase::kIdle;
      return;
    }

    mapper_->start(
      Eigen::Vector3d(
        latest_frame_.position.x, latest_frame_.position.y, latest_frame_.position.z),
      Eigen::Quaterniond(
        latest_frame_.quaternion.w, latest_frame_.quaternion.x, latest_frame_.quaternion.y,
        latest_frame_.quaternion.z).normalized());
    filter_->reset();
    clearFollowState();
    phase_ = Phase::kFollowing;
    RCLCPP_INFO(get_logger(), "start pose reached: following the controller");
  }

  void stepFollowing(double dt)
  {
    // One solve per headset frame, not one per control cycle.
    //
    // The Cartesian target only changes when a frame arrives, but the solver was
    // re-run every cycle against it, seeded from the measured joints. A solver
    // that is not deterministic - and the SNS plugin is not, its search restarts
    // within the 7 axis null space - then returns a slightly different answer
    // for the same target each time, a joint space wobble of a few hundredths of
    // a radian per cycle. The controller reports that wobble as a velocity,
    // (q - q_prev) / dt, so 0.01 rad at 100 Hz is 1.0 rad/s: the velocity
    // interface saturates on a hand that is not moving at all. Between frames
    // there is nothing new to solve for, so the last solution is republished
    // instead - the firmware's watchdog wants a continuous stream, not a new
    // target - and the joint filter keeps easing toward it, which is also what
    // turns one frame's step into a short ramp instead of a step.
    const bool new_frame = latest_frame_valid_ && frame_sequence_ != processed_sequence_;
    if (new_frame) {
      processed_sequence_ = frame_sequence_;

      Eigen::Vector3d ee_position;
      Eigen::Matrix3d ee_rotation;
      if (!planner_->forwardKinematics(measured_, ee_position, ee_rotation)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "forward kinematics failed; holding");
        publishHold();
        return;
      }

      // The pose the increment is added to. `measured` closes the mapping
      // around where the arm actually is, so a target that was clipped or
      // refused is not carried into the next frame. `commanded` accumulates on
      // the last command instead, which is what makes the arm deliver the
      // operator's whole displacement even while it lags behind.
      const Eigen::Vector3d measured_tcp = ee_position + ee_rotation * tcp_offset_;
      Eigen::Vector3d reference_position = measured_tcp;
      Eigen::Matrix3d reference_rotation = ee_rotation;
      if (reference_from_command_ && have_reference_pose_) {
        reference_position = reference_pose_.position;
        reference_rotation = reference_pose_.rotation;
      }

      const CartesianTarget target = mapper_->feed(
        Eigen::Vector3d(
          latest_frame_.position.x, latest_frame_.position.y, latest_frame_.position.z),
        Eigen::Quaterniond(
          latest_frame_.quaternion.w, latest_frame_.quaternion.x, latest_frame_.quaternion.y,
          latest_frame_.quaternion.z).normalized(),
        reference_position, reference_rotation);

      if (target.valid) {
        // Bounded lead first: in the commanded mode this is what keeps an
        // unreachable target from collecting the operator's motion forever.
        const CartesianTarget bounded =
          clampTargetLead(target, measured_tcp, ee_rotation, max_lead_m_, max_lead_rad_);
        // Then the deadband: below it the target is kept as it is, so a still
        // hand produces no new solve. See targetDiffers for what a deadband
        // costs in the measured mode, which is why it is off by default.
        if (!have_cartesian_target_ ||
          targetDiffers(bounded, cartesian_target_, target_deadband_m_, target_deadband_rad_))
        {
          cartesian_target_ = bounded;
          have_cartesian_target_ = true;
          // The cached solution belongs to the target it was solved for.
          have_solution_ = false;
        }
        reference_pose_ = cartesian_target_;
        have_reference_pose_ = true;
      }
    }

    if (!have_cartesian_target_) {
      publishHold();
      return;
    }

    if (!have_solution_) {
      // Seeded with the previous solution when there is one. That matters more
      // than it looks: with the same target and the same seed the solver returns
      // the same answer, so a target that is not moving produces a command that
      // is not moving either, and the redundant axis stays where the last
      // solution put it instead of wandering through the null space.
      const Eigen::VectorXd & seed = have_commanded_joints_ ? commanded_joints_ : measured_;
      const TeleopPlanner::Solution solution =
        planner_->solve(
        cartesian_target_.position - cartesian_target_.rotation * tcp_offset_,
        cartesian_target_.rotation, seed);

      if (!solution.converged) {
        // Holding beats commanding a solution that does not reach the target:
        // the remaining error would point in a direction the operator did not
        // ask for.
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "target unreachable (position off by %.4f m, rotation off by %.4f rad, minimum singular "
          "value %.4f); holding",
          solution.position_residual, solution.rotation_residual,
          planner_->minimumSingularValue(measured_));
        publishHold();
        return;
      }

      commanded_joints_ = solution.joints;
      have_commanded_joints_ = true;
      have_solution_ = true;
    }

    publishPositions(filter_->apply(commanded_joints_, measured_, dt));
  }

  /// Forget the follow state: the cached target, the cached solution and the
  /// pose the mapping accumulates on. Every transition out of following calls
  /// this, because a solution solved for a pose the arm has since left is not a
  /// solution for this one.
  void clearFollowState()
  {
    have_cartesian_target_ = false;
    have_solution_ = false;
    have_commanded_joints_ = false;
    have_reference_pose_ = false;
  }

  // ---------------------------------------------------------------- publishing

  void publishHold()
  {
    // A hold is published rather than implied by silence. Silence would leave the
    // controller writing its last setpoint until its own staleness timeout, which
    // is a fraction of a second of travel nobody asked for.
    publishPositions(measured_);
  }

  void publishPositions(const Eigen::VectorXd & positions)
  {
    if (command_publisher_ == nullptr ||
      positions.size() != static_cast<Eigen::Index>(joint_names_.size()))
    {
      return;
    }
    std_msgs::msg::Float64MultiArray message;
    message.data.reserve(joint_names_.size());
    for (Eigen::Index i = 0; i < positions.size(); ++i) {
      message.data.push_back(positions(i));
    }
    command_publisher_->publish(message);
  }

  // ------------------------------------------------------------------ gripper

  void sendGripperGoal(double position, const char * what)
  {
    if (!gripper_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "the gripper action '%s' is not available; the gripper did not %s",
        gripper_action_.c_str(), what);
      return;
    }

    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = gripper_max_effort_;

    // Fire and forget. The result is not waited for: a grasp that stalls against
    // an object is a success for this gripper, and reporting every one of those
    // would fill the log with non-events. Only the acknowledgement is
    // interesting, and only when it is a refusal.
    rclcpp_action::Client<GripperCommand>::SendGoalOptions options;
    options.goal_response_callback =
      [this, what, position](const GripperCommandGoalHandle::SharedPtr & handle) {
        if (handle == nullptr) {
          RCLCPP_WARN(get_logger(), "the gripper did not %s: the goal was rejected", what);
          return;
        }
        RCLCPP_INFO(get_logger(), "gripper %s (target %.5f m)", what, position);
      };
    gripper_client_->async_send_goal(goal, options);
  }

  // ------------------------------------------------------------------- members

  // Parameters.
  std::string frame_topic_;
  std::string joint_state_topic_;
  std::string command_topic_;
  double control_rate_hz_ = 100.0;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd start_pose_;
  double start_pose_speed_ = kStartPoseSpeed;
  double start_pose_accel_ = kStartPoseAccel;
  TeleopLimits teleop_limits_;
  /// Frame the controller's rotation increment is read in, and the per axis
  /// signs applied to it. Both are settable at run time; see onParameterUpdate.
  std::string rotation_mode_name_ = "world";
  Eigen::Vector3d rotation_signs_ = Eigen::Vector3d::Ones();
  double headset_timeout_s_ = 0.5;
  bool gripper_enabled_ = true;
  std::string gripper_action_;
  double gripper_open_position_ = 0.07479;
  double gripper_close_position_ = 0.0;
  double gripper_max_effort_ = 3.5;
  double trigger_threshold_ = 0.5;
  double trigger_debounce_s_ = 0.05;
  double min_close_s_ = 0.5;

  // Planning and filtering. Built in initialize().
  TeleopPlanner::Options planner_options_;
  std::unique_ptr<TeleopPlanner> planner_;
  std::unique_ptr<TeleopIncrementMapper> mapper_;
  std::unique_ptr<JointTargetFilter> filter_;
  bool planner_ready_ = false;
  std::string planner_error_;

  // State.
  Phase phase_ = Phase::kWaitingForState;
  Eigen::VectorXd measured_;
  bool have_measured_ = false;
  CartesianTarget cartesian_target_;
  /// tip_link -> TCP, in the tip frame (config/tcp.yaml).
  Eigen::Vector3d tcp_offset_ = Eigen::Vector3d(0.0, 0.0, 0.18);
  bool have_cartesian_target_ = false;
  /// The last joint solution, and whether there is one. Held between headset
  /// frames: re-solving an unchanged target every control cycle is what made the
  /// command wobble (see stepFollowing).
  Eigen::VectorXd commanded_joints_;
  bool have_commanded_joints_ = false;
  bool have_solution_ = false;
  /// The pose the increment is added to when `reference_pose` is `commanded`.
  CartesianTarget reference_pose_;
  bool have_reference_pose_ = false;
  /// Read once at startup: which pose the increment is measured against, how
  /// small a target change is not worth solving for, and how far the target may
  /// run ahead of the arm.
  bool reference_from_command_ = false;
  double target_deadband_m_ = 0.0;
  double target_deadband_rad_ = 0.0;
  double max_lead_m_ = 0.15;
  double max_lead_rad_ = 0.6;
  std::unique_ptr<JointMoveProfile> start_move_;

  msg::VrControllerFrame latest_frame_;
  bool latest_frame_valid_ = false;
  std::size_t frame_sequence_ = 0;
  std::size_t processed_sequence_ = 0;
  double last_frame_time_ = 0.0;
  double last_tick_ = steadySeconds();

  bool saw_grip_ = false;
  bool trigger_raw_ = false;
  bool trigger_stable_ = false;
  double trigger_changed_at_ = 0.0;
  double last_close_ = 0.0;

  // ROS interfaces. Built in initialize().
  rclcpp::Subscription<msg::VrControllerFrame>::SharedPtr frame_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  /// Keeps the run time mapping callback alive; dropping it unregisters it.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

}  // namespace litearm_vr_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Overrides are declared automatically because the kinematics solver this node
  // loads reads its own tuning as ordinary parameters (`litearm_arm.solver_type`
  // and friends) and checks for them with has_parameter. Without this flag the
  // solver would silently fall back to its built-in defaults and the
  // configuration file would appear to have no effect. The cost is that every
  // parameter read in this node has to go through the guarded helper above.
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<litearm_vr_teleop::VrTeleopNode>(options);
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
