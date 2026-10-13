// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.
//
// Planning for teleoperation: forward kinematics, the Jacobian, and two
// interchangeable inverse kinematics backends.

#ifndef LITEARM_VR_TELEOP__TELEOP_PLANNER_HPP_
#define LITEARM_VR_TELEOP__TELEOP_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_loader.hpp>

namespace litearm_vr_teleop
{

/// Robot model plus an inverse kinematics solver, with the solver swappable.
///
/// Two backends exist because they answer different questions and neither wins
/// outright:
///
///   - the plugin backend reuses whatever solver the MoveIt configuration
///     names, here `sns_ik_kinematics_plugin::SnsIkKinematicsPlugin` with the
///     measured `solver_type` / `loop_period` / `eps` / `timeout` from
///     `config/kinematics.yaml`. Reusing it means the teleoperation path and the
///     planning path solve the same problem the same way, and a solver change is
///     a configuration change. The cost is that its latency is not under this
///     node's control: the configuration budgets 0.2 s per call, far above a
///     10 ms control cycle.
///
///   - the damped least squares backend is implemented here, on top of the
///     forward kinematics and the Jacobian that the MoveIt core already
///     provides. It reproduces the reference implementation's arithmetic
///     (seed at the measured joints, a damping term instead of inverting a
///     singular Jacobian, null space pull back toward the seed so the redundant
///     axis has a defined answer) and its acceptance test (re-run forward
///     kinematics and compare position and rotation residuals). Its latency is
///     predictable, which is the whole reason to prefer it on a following task.
///
/// Both backends are verified the same way, by measuring the residual of the
/// returned solution rather than by trusting the solver's own success flag.
class TeleopPlanner
{
public:
  enum class IkBackend
  {
    kPlugin,
    kDls,
  };

  struct Options
  {
    IkBackend backend = IkBackend::kDls;
    /// Planning group, and the prefix under which the solver reads its own
    /// parameters (MoveIt's convention: `litearm_arm.kinematics_solver`).
    std::string group_name = "litearm_arm";
    /// The frame the teleoperated pose is expressed in, and the group's tip.
    std::string base_frame = "base_link";
    std::string tip_link = "ee_link";
    std::string robot_description = "robot_description";
    /// The SRDF, which is what names the planning group. MoveIt's model cannot be
    /// built without it, and without it there is no group to plan for.
    std::string robot_description_semantic = "robot_description_semantic";
    /// Acceptance of a solution, measured by forward kinematics on it. Ported
    /// from the reference: 3 mm and 0.05 rad.
    double position_tolerance = 3e-3;
    double rotation_tolerance = 0.05;
    /// Damped least squares parameters.
    double dls_damping = 0.05;
    double dls_null_space_gain = 0.2;
    int dls_max_iterations = 120;
    double dls_max_step_rad = 0.2;
    /// A Jacobian minimum singular value below this is logged as a singularity
    /// warning. It is a diagnostic threshold, not a limit: the arm is allowed to
    /// work near a singularity, the operator is just told that the solution is
    /// ill conditioned there.
    double singularity_warning_threshold = 0.02;
  };

  struct Solution
  {
    Eigen::VectorXd joints;
    double position_residual = 0.0;
    double rotation_residual = 0.0;
    /// True only when the residual of the RETURNED solution is inside tolerance,
    /// whatever the solver itself reported.
    bool converged = false;
  };

  TeleopPlanner(Options options, rclcpp::Node::SharedPtr node);
  ~TeleopPlanner();

  TeleopPlanner(const TeleopPlanner &) = delete;
  TeleopPlanner & operator=(const TeleopPlanner &) = delete;

  /// Load the robot model and, for the plugin backend, create the solver.
  /// Returns false and fills `error` with something actionable on failure.
  bool initialize(std::string & error);

  const std::string & backendName() const;

  std::size_t dof() const { return joint_names_.size(); }
  const std::vector<std::string> & jointNames() const { return joint_names_; }
  Eigen::VectorXd lowerLimits() const { return lower_limits_; }
  Eigen::VectorXd upperLimits() const { return upper_limits_; }

  /// Forward kinematics of the group tip. Not const: it uses the shared scratch
  /// robot state, which is why this class is single threaded by contract (the
  /// node runs one executor with one callback at a time).
  bool forwardKinematics(
    const Eigen::VectorXd & joints, Eigen::Vector3d & position, Eigen::Matrix3d & rotation);

  /// Solve for the joint values reaching `position` / `rotation`, seeded at
  /// `seed`.
  Solution solve(
    const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation, const Eigen::VectorXd & seed);

  /// Solve with the damped backend whatever the configuration names.
  ///
  /// Used when the arm is near a singularity. The configured plugin's solution
  /// jumps between configurations there - a few hundredths of a radian of hand
  /// motion can move a joint by half a radian, and the arm is as likely to go up
  /// as down - while a damped solve stays continuous through the same region, at
  /// the cost of a residual it reports honestly.
  Solution solveDamped(
    const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation, const Eigen::VectorXd & seed);

  /// Smallest singular value of the group Jacobian at `joints`.
  ///
  /// Reported so that a failing inverse kinematics call can be attributed to
  /// conditioning rather than to an unreachable target. The reference prints the
  /// same quantity in its warning.
  double minimumSingularValue(const Eigen::VectorXd & joints);

  /// Joint speeds that produce this tool twist, by damped least squares.
  ///
  /// The velocity following mode's replacement for `solve`, and the step MoveIt
  /// Servo takes on every cycle of its loop: the Jacobian is what maps a joint
  /// speed to a tool speed, so a twist is what it answers directly, with no
  /// iteration and no seed and therefore no dependence on where the solver
  /// happens to start. On this machine that is the difference between a
  /// millisecond and the tens of milliseconds a plugin takes, which is what lets
  /// the following loop run at the control rate instead of around a solver.
  ///
  /// The damping has the same job it has in `solveDamped`, and the same cost:
  /// near a singularity `J J^T` is inverted against a small number, and without
  /// damping a bounded tool speed would ask for an unbounded joint speed.
  ///
  /// `joints` is where the Jacobian is evaluated - the measured joints, as Servo
  /// uses - and the result is in the group's variable order. A twist that cannot
  /// be answered at all gives zeros, which is a hold rather than a guess.
  Eigen::VectorXd twistToJointVelocity(
    const Eigen::Vector3d & linear, const Eigen::Vector3d & angular,
    const Eigen::VectorXd & joints, double damping);

private:
  /// Read a description parameter, declaring it if the launch file did not.
  std::string describeAndRead(const std::string & name, std::string & error) const;

  bool loadModel(std::string & error);
  bool loadSolverPlugin(std::string & error);
  void collectBounds();

  /// Write `joints` into the scratch state and refresh the link transforms.
  void setScratchState(const Eigen::VectorXd & joints);

  Eigen::VectorXd clampToLimits(const Eigen::VectorXd & joints) const;

  /// Measure how far the solution actually lands from the request.
  void measure(
    const Eigen::VectorXd & joints, const Eigen::Vector3d & position,
    const Eigen::Matrix3d & rotation, double & position_residual, double & rotation_residual);

  Solution solveWithPlugin(
    const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation, const Eigen::VectorXd & seed);

  Solution solveWithDls(
    const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation, const Eigen::VectorXd & seed);

  Options options_;
  rclcpp::Node::SharedPtr node_;

  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup * group_ = nullptr;
  std::unique_ptr<moveit::core::RobotState> scratch_;
  const moveit::core::LinkModel * tip_ = nullptr;

  std::vector<std::string> joint_names_;
  Eigen::VectorXd lower_limits_;
  Eigen::VectorXd upper_limits_;

  std::unique_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> solver_loader_;
  std::shared_ptr<kinematics::KinematicsBase> solver_;
  /// Maps the solver's joint order onto this group's variable order. Both are
  /// `joint1..joint7` today, but a solver that reports a different order would
  /// otherwise move the wrong axis, which is a failure that is very hard to read
  /// from the outside.
  std::vector<Eigen::Index> solver_to_group_;
};

}  // namespace litearm_vr_teleop

#endif  // LITEARM_VR_TELEOP__TELEOP_PLANNER_HPP_
