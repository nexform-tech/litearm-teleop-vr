// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.

#include "litearm_vr_teleop/teleop_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace litearm_vr_teleop
{
namespace
{

/// Rotation vector of a rotation matrix, with the identity handled.
///
/// The residual check needs this for the difference between the requested and
/// the achieved orientation, which is routinely exactly the identity when a
/// solver lands on the target.
Eigen::Vector3d toRotationVector(const Eigen::Matrix3d & rotation)
{
  const Eigen::AngleAxisd angle_axis(rotation);
  return angle_axis.axis() * angle_axis.angle();
}

Eigen::Quaterniond toQuaternion(const Eigen::Matrix3d & rotation)
{
  return Eigen::Quaterniond(rotation).normalized();
}

/// Turn a list into a readable comma separated string for error messages.
std::string join(const std::vector<std::string> & items)
{
  std::ostringstream stream;
  for (std::size_t i = 0; i < items.size(); ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << items[i];
  }
  return stream.str();
}

}  // namespace

TeleopPlanner::TeleopPlanner(Options options, rclcpp::Node::SharedPtr node)
: options_(std::move(options)), node_(std::move(node))
{
}

TeleopPlanner::~TeleopPlanner() = default;

const std::string & TeleopPlanner::backendName() const
{
  static const std::string kPluginName = "plugin";
  static const std::string kDlsName = "dls";
  return options_.backend == IkBackend::kPlugin ? kPluginName : kDlsName;
}

std::string TeleopPlanner::describeAndRead(const std::string & name, std::string & error) const
{
  if (!node_->has_parameter(name)) {
    node_->declare_parameter<std::string>(name, "");
  }
  const std::string value = node_->get_parameter(name).as_string();
  if (value.empty()) {
    error = "parameter '" + name + "' is empty; planning needs both the robot description "
            "and the semantic description";
  }
  return value;
}

bool TeleopPlanner::initialize(std::string & error)
{
  if (!loadModel(error)) {
    return false;
  }
  collectBounds();

  if (options_.backend == IkBackend::kPlugin) {
    return loadSolverPlugin(error);
  }

  // The damped least squares backend needs nothing beyond the model: forward
  // kinematics and the Jacobian both come from the MoveIt core.
  return true;
}

bool TeleopPlanner::loadModel(std::string & error)
{
  const auto logger = node_->get_logger();

  // Both descriptions normally arrive as launch overrides, and an override is only
  // visible to a node that declares the parameter. Declaring them here with an
  // empty fallback makes the override readable, and an empty value is then the
  // error rather than a mystery about where the description went.
  const std::string description = describeAndRead(options_.robot_description, error);
  if (!error.empty()) {
    return false;
  }
  const std::string semantic = describeAndRead(options_.robot_description_semantic, error);
  if (!error.empty()) {
    return false;
  }

  // The model is built from the two strings directly rather than through
  // RobotModelLoader, which would read the same two parameters but also drag in
  // the planning pipeline's library and a node's worth of hidden parameter
  // lookups. This class needs a kinematic model and a named group, and nothing
  // else.
  const urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(description);
  if (urdf_model == nullptr) {
    error = "the robot description in '" + options_.robot_description + "' is not valid URDF";
    return false;
  }

  auto srdf_model = std::make_shared<srdf::Model>();
  if (!srdf_model->initString(*urdf_model, semantic)) {
    error = "the semantic description in '" + options_.robot_description_semantic +
            "' is not valid SRDF";
    return false;
  }

  model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  if (model_ == nullptr) {
    error = "the robot model could not be built from the URDF and SRDF pair";
    return false;
  }

  group_ = model_->getJointModelGroup(options_.group_name);
  if (group_ == nullptr) {
    error = "planning group '" + options_.group_name + "' does not exist; the model offers: " +
            join(model_->getJointModelGroupNames());
    return false;
  }

  tip_ = model_->getLinkModel(options_.tip_link);
  if (tip_ == nullptr) {
    error = "tip link '" + options_.tip_link + "' does not exist in the robot model";
    return false;
  }

  joint_names_ = group_->getVariableNames();
  if (joint_names_.empty()) {
    error = "planning group '" + options_.group_name + "' has no variables";
    return false;
  }

  scratch_ = std::make_unique<moveit::core::RobotState>(model_);
  scratch_->setToDefaultValues();

  // The solver is told the base frame explicitly, but forward kinematics and
  // the Jacobian come back in the model frame. If the two differ the residuals
  // would be measured in a frame nobody asked about, so say so instead of
  // silently measuring the wrong thing.
  if (options_.base_frame != model_->getModelFrame()) {
    RCLCPP_WARN(
      logger,
      "base_frame '%s' differs from the model frame '%s'; forward kinematics and the "
      "Jacobian are expressed in the model frame, so the planned pose must be too",
      options_.base_frame.c_str(), model_->getModelFrame().c_str());
  }

  return true;
}

void TeleopPlanner::collectBounds()
{
  const auto logger = node_->get_logger();

  lower_limits_ = Eigen::VectorXd::Constant(
    static_cast<Eigen::Index>(joint_names_.size()), std::numeric_limits<double>::lowest());
  upper_limits_ = Eigen::VectorXd::Constant(
    static_cast<Eigen::Index>(joint_names_.size()), std::numeric_limits<double>::max());

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const moveit::core::VariableBounds & bounds = model_->getVariableBounds(joint_names_[i]);
    if (!bounds.position_bounded_) {
      // A variable without position limits is legitimate (a continuous joint,
      // for instance). Treating the default 0.0 that MoveIt stores as the limit
      // would clamp that joint to zero, which is a way to destroy an arm.
      RCLCPP_DEBUG(
        logger, "joint variable '%s' has no position bounds, leaving it unbounded",
        joint_names_[i].c_str());
      continue;
    }
    lower_limits_(i) = bounds.min_position_;
    upper_limits_(i) = bounds.max_position_;
  }
}

bool TeleopPlanner::loadSolverPlugin(std::string & error)
{
  const auto logger = node_->get_logger();
  const std::string solver_parameter = options_.group_name + ".kinematics_solver";

  if (!node_->has_parameter(solver_parameter)) {
    node_->declare_parameter<std::string>(solver_parameter, "");
  }
  const std::string solver_class = node_->get_parameter(solver_parameter).as_string();
  if (solver_class.empty()) {
    error = "parameter '" + solver_parameter +
            "' is empty; the plugin backend needs a solver class name (see the MoveIt "
            "configuration's kinematics.yaml), or select the 'dls' backend which needs no plugin";
    return false;
  }

  try {
    // The base class is exported by moveit_core; the solver itself is looked up
    // by the name the configuration gives, so switching solvers stays a
    // configuration change and this package needs no build dependency on the
    // solver it happens to run with.
    solver_loader_ =
      std::make_unique<pluginlib::ClassLoader<kinematics::KinematicsBase>>(
      "moveit_core", "kinematics::KinematicsBase");
    solver_ = solver_loader_->createSharedInstance(solver_class);
  } catch (const std::exception & exception) {
    error = "could not load kinematics solver '" + solver_class + "': " + exception.what();
    return false;
  }

  const std::vector<std::string> tips{options_.tip_link};
  if (!solver_->initialize(
      node_, *model_, options_.group_name, options_.base_frame, tips, 0.0))
  {
    error = "kinematics solver '" + solver_class + "' refused to initialize for group '" +
            options_.group_name + "' from '" + options_.base_frame + "' to '" +
            options_.tip_link + "'";
    return false;
  }

  // The solver returns its joints in its own order. Map it onto the group's
  // variable order once, here, so the rest of the code never has to think about
  // it. A solver reporting a joint the group does not have is a configuration
  // error worth stopping for.
  const std::vector<std::string> & solver_joints = solver_->getJointNames();
  solver_to_group_.clear();
  solver_to_group_.reserve(solver_joints.size());
  for (const std::string & solver_joint : solver_joints) {
    const auto found = std::find(joint_names_.begin(), joint_names_.end(), solver_joint);
    if (found == joint_names_.end()) {
      error = "kinematics solver '" + solver_class + "' reports joint '" + solver_joint +
              "', which is not part of group '" + options_.group_name + "'";
      return false;
    }
    solver_to_group_.push_back(std::distance(joint_names_.begin(), found));
  }
  if (solver_to_group_.size() != joint_names_.size()) {
    error = "kinematics solver '" + solver_class + "' reports " +
            std::to_string(solver_to_group_.size()) + " joints while group '" +
            options_.group_name + "' has " + std::to_string(joint_names_.size());
    return false;
  }

  RCLCPP_INFO(
    logger, "planning backend 'plugin': %s for group %s (%zu joints)",
    solver_class.c_str(), options_.group_name.c_str(), joint_names_.size());
  return true;
}

void TeleopPlanner::setScratchState(const Eigen::VectorXd & joints)
{
  scratch_->setJointGroupPositions(group_, joints);
  scratch_->updateLinkTransforms();
}

Eigen::VectorXd TeleopPlanner::clampToLimits(const Eigen::VectorXd & joints) const
{
  return joints.cwiseMax(lower_limits_).cwiseMin(upper_limits_);
}

bool TeleopPlanner::forwardKinematics(
  const Eigen::VectorXd & joints, Eigen::Vector3d & position, Eigen::Matrix3d & rotation)
{
  if (joints.size() != static_cast<Eigen::Index>(joint_names_.size())) {
    return false;
  }
  setScratchState(joints);
  const Eigen::Isometry3d & transform = scratch_->getGlobalLinkTransform(tip_);
  position = transform.translation();
  rotation = transform.linear();
  return true;
}

void TeleopPlanner::measure(
  const Eigen::VectorXd & joints, const Eigen::Vector3d & position,
  const Eigen::Matrix3d & rotation, double & position_residual, double & rotation_residual)
{
  Eigen::Vector3d reached_position;
  Eigen::Matrix3d reached_rotation;
  forwardKinematics(joints, reached_position, reached_rotation);

  position_residual = (reached_position - position).norm();
  rotation_residual = toRotationVector(rotation.transpose() * reached_rotation).norm();
}

TeleopPlanner::Solution TeleopPlanner::solveDamped(
  const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation, const Eigen::VectorXd & seed)
{
  return solveWithDls(position, rotation, seed);
}

double TeleopPlanner::minimumSingularValue(const Eigen::VectorXd & joints)
{
  if (joints.size() != static_cast<Eigen::Index>(joint_names_.size())) {
    return 0.0;
  }
  setScratchState(joints);
  const Eigen::MatrixXd jacobian = scratch_->getJacobian(group_);
  if (jacobian.size() == 0) {
    return 0.0;
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  return svd.singularValues().minCoeff();
}

TeleopPlanner::Solution TeleopPlanner::solve(
  const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation, const Eigen::VectorXd & seed)
{
  if (seed.size() != static_cast<Eigen::Index>(joint_names_.size())) {
    Solution refused;
    refused.joints = seed;
    return refused;
  }
  const Eigen::VectorXd clamped_seed = clampToLimits(seed);

  Solution solution = options_.backend == IkBackend::kPlugin ?
    solveWithPlugin(position, rotation, clamped_seed) :
    solveWithDls(position, rotation, clamped_seed);

  solution.joints = clampToLimits(solution.joints);
  // Measure after clamping: a solution that was valid and then got clamped is no
  // longer valid, and reporting it as converged would hide exactly the case that
  // matters (a target just outside the joint range).
  measure(
    solution.joints, position, rotation, solution.position_residual, solution.rotation_residual);
  solution.converged =
    solution.position_residual < options_.position_tolerance &&
    solution.rotation_residual < options_.rotation_tolerance;
  return solution;
}

TeleopPlanner::Solution TeleopPlanner::solveWithPlugin(
  const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation, const Eigen::VectorXd & seed)
{
  Solution solution;
  solution.joints = seed;
  if (solver_ == nullptr) {
    return solution;
  }

  geometry_msgs::msg::Pose target;
  target.position.x = position.x();
  target.position.y = position.y();
  target.position.z = position.z();
  const Eigen::Quaterniond orientation = toQuaternion(rotation);
  target.orientation.x = orientation.x();
  target.orientation.y = orientation.y();
  target.orientation.z = orientation.z();
  target.orientation.w = orientation.w();

  // Reorder the measured seed into the solver's joint order.
  std::vector<double> seed_in_solver_order(solver_to_group_.size(), 0.0);
  for (std::size_t i = 0; i < solver_to_group_.size(); ++i) {
    seed_in_solver_order[i] = seed(solver_to_group_[i]);
  }

  std::vector<double> raw_solution;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  // A single attempt, seeded at the measured joints. The configuration's
  // `kinematics_solver_attempts` is a move_group notion of retrying from
  // randomised seeds; for a following task the measured state is the best seed
  // there is, and a random restart would return a solution in a different
  // configuration branch, which for a redundant arm means a large, unexpected
  // motion.
  if (!solver_->getPositionIK(target, seed_in_solver_order, raw_solution, error_code)) {
    return solution;
  }
  if (raw_solution.size() != solver_to_group_.size()) {
    return solution;
  }

  for (std::size_t i = 0; i < solver_to_group_.size(); ++i) {
    solution.joints(solver_to_group_[i]) = raw_solution[i];
  }
  return solution;
}

TeleopPlanner::Solution TeleopPlanner::solveWithDls(
  const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation, const Eigen::VectorXd & seed)
{
  Solution solution;
  solution.joints = seed;

  const Eigen::Index dof = static_cast<Eigen::Index>(joint_names_.size());
  const double damping = std::max(1e-6, options_.dls_damping);
  const double damping_squared = damping * damping;

  Eigen::VectorXd joints = seed;

  for (int iteration = 0; iteration < options_.dls_max_iterations; ++iteration) {
    setScratchState(joints);
    const Eigen::Isometry3d & transform = scratch_->getGlobalLinkTransform(tip_);
    const Eigen::Vector3d current_position = transform.translation();
    const Eigen::Matrix3d current_rotation = transform.linear();

    const Eigen::Vector3d position_error = position - current_position;
    // Rotation error in the model frame; the ratio of the two rotations, not
    // their difference, so that a large yaw does not look like a small error
    // near pi.
    const Eigen::Vector3d rotation_error =
      toRotationVector(rotation * current_rotation.transpose());

    const double position_residual = position_error.norm();
    const double rotation_residual = rotation_error.norm();
    if (position_residual < options_.position_tolerance &&
      rotation_residual < options_.rotation_tolerance)
    {
      break;
    }

    const Eigen::MatrixXd jacobian = scratch_->getJacobian(group_);
    if (jacobian.rows() != 6 || jacobian.cols() != dof) {
      break;
    }

    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = position_error;
    error.tail<3>() = rotation_error;

    // Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 error.
    //
    // The damping is what makes this usable near a singularity, where J J^T is
    // singular and an undamped inverse would produce an unbounded joint
    // velocity for a bounded end effector correction. It trades exactness for
    // boundedness, which is the right trade when a human is holding the target.
    const Eigen::Matrix<double, 6, 6> jjt =
      jacobian * jacobian.transpose() +
      Eigen::Matrix<double, 6, 6>::Identity() * damping_squared;
    const Eigen::Matrix<double, 6, 6> jjt_inverse =
      jjt.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

    Eigen::VectorXd delta = jacobian.transpose() * (jjt_inverse * error);

    // Null space pull toward the seed. The arm has seven joints for six degrees
    // of freedom, so one direction of joint motion does not move the tool at
    // all. Without this term that direction is whatever the arithmetic happens
    // to produce, and the arm drifts into a different configuration between
    // frames. Pulling toward the seed keeps the redundant motion where it
    // started.
    if (options_.dls_null_space_gain > 0.0) {
      const Eigen::MatrixXd jacobian_pseudo_inverse = jacobian.transpose() * jjt_inverse;
      const Eigen::MatrixXd null_space_projector =
        Eigen::MatrixXd::Identity(dof, dof) - jacobian_pseudo_inverse * jacobian;
      delta += null_space_projector * (options_.dls_null_space_gain * (seed - joints));
    }

    // Bound one iteration's motion, then the joint range. The bound keeps a
    // single ill conditioned iteration from throwing the arm across its range
    // before the next iteration has a chance to correct.
    const double step_scale = delta.cwiseAbs().maxCoeff() > options_.dls_max_step_rad ?
      options_.dls_max_step_rad / delta.cwiseAbs().maxCoeff() : 1.0;
    joints = clampToLimits(joints + step_scale * delta);
  }

  solution.joints = joints;
  return solution;
}

}  // namespace litearm_vr_teleop
