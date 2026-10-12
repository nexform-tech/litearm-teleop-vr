// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.

#include "litearm_vr_teleop/teleop_math.hpp"

#include <algorithm>
#include <cmath>

namespace litearm_vr_teleop
{
namespace
{

/// Clamp a cycle time into a sane band.
///
/// A zero or negative dt would divide the step limit to nothing and stop the
/// arm; a very large one (the process was descheduled, the headset paused for a
/// second) would authorise a correspondingly huge step. Both cases are real on
/// a workstation, so the band is applied here instead of trusting the caller.
/// The band is the one the reference implementation used.
double sanitiseDeltaSeconds(double dt)
{
  return std::min(0.05, std::max(1e-3, dt));
}

/// Rotation vector to rotation matrix, with the degenerate case handled.
///
/// Eigen::AngleAxisd cannot be constructed from a zero rotation vector: it would
/// divide the vector by its own (zero) norm. A zero increment is the normal case
/// whenever the operator holds the controller still, so this cannot be left to
/// luck.
Eigen::Matrix3d rotationVectorToMatrix(const Eigen::Vector3d & rotation_vector)
{
  const double angle = rotation_vector.norm();
  if (angle < 1e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
}

/// Rotation matrix to rotation vector. Eigen's AngleAxisd yields an angle in
/// [0, pi], the same branch scipy's `Rotation.as_rotvec()` uses in the reference
/// implementation, so the two agree for the full range the headset can produce.
Eigen::Vector3d matrixToRotationVector(const Eigen::Matrix3d & rotation)
{
  const Eigen::AngleAxisd angle_axis(rotation);
  return angle_axis.axis() * angle_axis.angle();
}

}  // namespace

AxisMapping singleArmUprightMapping()
{
  AxisMapping mapping;

  // The tool takes the controller's rotation in the world. No axis
  // correspondence is involved and none may be added: this mode is what makes
  // the response independent of the arm's pose and of how the controller is
  // held. See RotationMapping in the header for why that matters here.
  mapping.rotation_mode = RotationMapping::kWorld;

  // Kept for kTool, which is the previous behaviour: the tool copies the
  // controller axis for axis. Identity here because the frames agree in axis
  // once the per axis signs below are applied.
  mapping.axis_remap <<
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0;

  // Measured correspondence between the controller frame (x right, y up, z back)
  // and the upright arm's base frame (x right, y forward, z up, the operator
  // facing +y):
  //   controller right (+x)  -> base +x, the operator's right
  //   controller up    (+y)  -> base up (+z)
  //   controller back  (+z)  -> base back (-y), hence the negated last row.
  mapping.translation_remap <<
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, -1.0, 0.0;

  // No correction. kWorld is geometric: the increment it applies is measured in
  // the robot frame and applied to the tool in the robot frame, so there is
  // nothing left to flip. The signs stay available because a rig whose headset
  // frame is mirrored, or a controller held upside down out of habit, is not a
  // geometry error and cannot be fixed by geometry.
  mapping.rotation_signs = Eigen::Vector3d::Ones();

  // What kTool adds on top: the reference's measured per axis signs, kept so
  // that asking for the previous behaviour gets the measured one and not a
  // tool frame mode with its signs left out.
  mapping.tool_rotation_signs = Eigen::Vector3d(1.0, -1.0, -1.0);

  return mapping;
}

TeleopIncrementMapper::TeleopIncrementMapper(AxisMapping mapping, TeleopLimits limits)
: mapping_(mapping), limits_(limits)
{
  // A zero frame window would make the average divide by zero and never smooth
  // anything. Treat it as the smallest window that still averages.
  limits_.moving_average_frames = std::max<std::size_t>(1, limits_.moving_average_frames);
}

bool targetDiffers(
  const CartesianTarget & candidate, const CartesianTarget & current,
  double position_deadband, double rotation_deadband)
{
  const double position_step = (candidate.position - current.position).norm();
  const double rotation_step =
    matrixToRotationVector(current.rotation.transpose() * candidate.rotation).norm();
  return position_step > std::max(0.0, position_deadband) ||
         rotation_step > std::max(0.0, rotation_deadband);
}

CartesianTarget clampTargetLead(
  const CartesianTarget & target, const Eigen::Vector3d & measured_position,
  const Eigen::Matrix3d & measured_rotation, double max_lead_position,
  double max_lead_rotation)
{
  CartesianTarget clamped = target;

  const Eigen::Vector3d lead = target.position - measured_position;
  if (max_lead_position > 0.0 && lead.norm() > max_lead_position) {
    clamped.position = measured_position + lead.normalized() * max_lead_position;
  }

  const Eigen::Vector3d rotation_lead =
    matrixToRotationVector(measured_rotation.transpose() * target.rotation);
  if (max_lead_rotation > 0.0 && rotation_lead.norm() > max_lead_rotation) {
    clamped.rotation = measured_rotation *
      rotationVectorToMatrix(rotation_lead.normalized() * max_lead_rotation);
  }

  return clamped;
}

void TeleopIncrementMapper::setMapping(const AxisMapping & mapping)
{
  mapping_ = mapping;
  // The window holds increments measured in the frame the previous mapping read.
  // Averaging the two frames together would apply a motion that is neither
  // mapping for as long as the window takes to drain.
  history_.clear();
}

Eigen::Matrix4d TeleopIncrementMapper::poseToHomogeneous(
  const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = orientation.normalized().toRotationMatrix();
  transform.block<3, 1>(0, 3) = position;
  return transform;
}

void TeleopIncrementMapper::start(
  const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation)
{
  active_ = true;
  previous_controller_ = poseToHomogeneous(position, orientation);
  // Clearing the history is what makes the anchoring frame produce no
  // increment: the average of a one element window is that element, and the
  // next frame's average is dragged toward zero by however many frames are in
  // flight. Starting with an empty window means the first following frame is
  // damped, which is exactly the behaviour wanted after a pause.
  history_.clear();
}

void TeleopIncrementMapper::pause()
{
  active_ = false;
}

void TeleopIncrementMapper::reset()
{
  active_ = false;
  previous_controller_ = Eigen::Matrix4d::Identity();
  history_.clear();
}

CartesianTarget TeleopIncrementMapper::feed(
  const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation,
  const Eigen::Vector3d & ee_position, const Eigen::Matrix3d & ee_rotation)
{
  CartesianTarget target;

  if (!active_) {
    // Paused. Nothing is commanded and nothing is recorded: `start()` clears the
    // history and re-anchors anyway, so accumulating increments here would only
    // be work whose result is thrown away.
    return target;
  }

  const Eigen::Matrix4d controller = poseToHomogeneous(position, orientation);

  // Relative motion of the controller between two frames. The inverse of a
  // rigid transform is its exact inverse, so the pseudoinverse the reference
  // reaches for is not needed here.
  const Eigen::Matrix4d delta = previous_controller_.inverse() * controller;
  const Eigen::Matrix3d previous_rotation = previous_controller_.block<3, 3>(0, 0);
  previous_controller_ = controller;

  // The headset's reference frame is gravity aligned but yawed with respect to
  // the robot, so every increment has to be carried across by the same
  // correspondence - translation and rotation alike. For a pure rotation the
  // carry is a similarity transform, because a rotation about an axis is a
  // rotation about the carried axis in the carried frame.
  const Eigen::Matrix3d controller_to_robot = mapping_.translation_remap.transpose();

  const Eigen::Vector3d relative_translation =
    controller_to_robot * delta.block<3, 1>(0, 3);

  Eigen::Matrix3d relative_rotation;
  if (mapping_.rotation_mode == RotationMapping::kWorld) {
    // The controller's rotation in the robot frame: take the rotation it had,
    // carry it across, and conjugate the body frame increment by it. Written
    // per frame rather than as a fixed matrix because the frame it is measured
    // in is the controller's own attitude, which changes with every frame.
    const Eigen::Matrix3d previous_in_robot = controller_to_robot * previous_rotation;
    relative_rotation = previous_in_robot * delta.block<3, 3>(0, 0) * previous_in_robot.transpose();
  } else {
    // The previous behaviour: the body frame increment is read as if the tool's
    // axes corresponded to the controller's, which is what axis_remap encodes.
    Eigen::Matrix4d rotation_frame = Eigen::Matrix4d::Identity();
    rotation_frame.block<3, 3>(0, 0) = mapping_.axis_remap;
    relative_rotation = (rotation_frame.inverse() * delta * rotation_frame).block<3, 3>(0, 0);
  }

  Eigen::Matrix<double, 6, 1> sample;
  sample.head<3>() = relative_translation;
  sample.tail<3>() = matrixToRotationVector(relative_rotation);

  history_.push_back(sample);
  while (history_.size() > limits_.moving_average_frames) {
    history_.pop_front();
  }

  Eigen::Matrix<double, 6, 1> averaged = Eigen::Matrix<double, 6, 1>::Zero();
  for (const auto & entry : history_) {
    averaged += entry;
  }
  averaged /= static_cast<double>(history_.size());

  const Eigen::Vector3d relative_position = averaged.head<3>();
  const Eigen::Vector3d signs =
    mapping_.rotation_mode == RotationMapping::kTool ?
    mapping_.tool_rotation_signs.cwiseProduct(mapping_.rotation_signs) :
    mapping_.rotation_signs;
  const Eigen::Vector3d relative_rotation_vector = averaged.tail<3>().cwiseProduct(signs);

  target.position = ee_position + relative_position;
  // Left multiplication under kWorld, right multiplication under kTool, and the
  // difference is the whole point of the two modes: rotating the measured pose
  // in the robot frame is a motion in the world, rotating it in the tool frame
  // is a motion about the tool's own axes. Under kWorld the tool therefore turns
  // about the same world axis as the controller whatever pose the arm is in;
  // under kTool it copies the hand axis for axis, which is the same motion only
  // at the one arm pose the signs were measured at.
  if (mapping_.rotation_mode == RotationMapping::kWorld) {
    target.rotation = rotationVectorToMatrix(relative_rotation_vector) * ee_rotation;
  } else {
    target.rotation = ee_rotation * rotationVectorToMatrix(relative_rotation_vector);
  }
  target.valid = true;
  return target;
}

JointTargetFilter::JointTargetFilter(TeleopLimits limits, std::size_t dof)
: limits_(limits), low_passed_(Eigen::VectorXd::Zero(dof)), seeded_(false)
{
}

Eigen::VectorXd JointTargetFilter::apply(
  const Eigen::VectorXd & target, const Eigen::VectorXd & current, double dt)
{
  if (target.size() != current.size()) {
    // A mismatch means a planner or a configuration error upstream; refusing to
    // move is the only safe answer, and returning `current` makes that explicit.
    return current;
  }

  const double safe_dt = sanitiseDeltaSeconds(dt);

  // The filter runs on the REQUESTED STEP, not on the absolute target, and that
  // distinction is the whole of this function.
  //
  // Smoothing the absolute target and then stepping toward it makes the arm
  // advance by a fraction of what was asked for, because the next cycle's target
  // is one increment ahead of wherever the arm actually got to. On the mock
  // hardware a 12 cm controller move was delivered as 8 cm with a 20 ms time
  // constant, and the fraction moves with the time constant and the cycle rate,
  // so it is not even a constant the operator could learn. Filtering the step
  // instead leaves the gain at one: a stream of constant steps is tracked with
  // exactly that step once the filter settles, and what the filter removes is the
  // cycle to cycle jitter on the rate, which is where the noise actually is.
  const Eigen::VectorXd requested_step = target - current;
  if (!seeded_ || low_passed_.size() != requested_step.size()) {
    low_passed_ = requested_step;
    seeded_ = true;
  } else {
    const double alpha = safe_dt / (safe_dt + std::max(0.0, limits_.low_pass_tau));
    low_passed_ += alpha * (requested_step - low_passed_);
  }

  const double step_limit = std::min(limits_.max_step_rad, limits_.max_joint_velocity * safe_dt);
  return current + low_passed_.cwiseMax(-step_limit).cwiseMin(step_limit);
}

void JointTargetFilter::reset()
{
  seeded_ = false;
}

}  // namespace litearm_vr_teleop
