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

namespace
{

/// Tolerance on the tick boundary, in seconds.
///
/// The grid is absolute and the clock is floating point, so a caller asking at
/// exactly the nominal tick time can land a few femtoseconds on either side of
/// it: `0.05 + 0.01` is not `0.06`. Without a tolerance the tick that misses
/// waits for the next poll, so at a 1 ms poll one boundary is a period ten
/// percent long - and because the grid is absolute, the phase never recovers.
/// A nanosecond is far below any resolution this loop has and far above the
/// arithmetic noise.
constexpr double kTickTolerance = 1e-9;

}  // namespace

ControlTick::ControlTick(double rate_hz)
: period_(1.0 / std::max(1.0, rate_hz))
{
}

bool ControlTick::due(double now)
{
  if (!started_) {
    started_ = true;
    origin_ = now;
    ticks_ = 1;
    return false;
  }
  // The tick due at this instant is the first grid point after it. Asking for it
  // this way covers both cases at once: on time, where that point is the next
  // period, and late, where it is several periods on - and the periods in between
  // are skipped rather than replayed, which is the whole point of an absolute
  // grid for a loop that can be descheduled (see the class documentation).
  const long long due_ticks =
    static_cast<long long>(std::floor((now + kTickTolerance - origin_) / period_)) + 1;
  if (due_ticks <= ticks_) {
    return false;
  }
  ticks_ = due_ticks;
  return true;
}

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
  limits_.moving_average_samples = std::max<std::size_t>(1, limits_.moving_average_samples);
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

Eigen::Matrix<double, 6, 1> TeleopIncrementMapper::relativeMotion(
  const Eigen::Matrix4d & controller)
{
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

  Eigen::Matrix<double, 6, 1> motion;
  motion.head<3>() = controller_to_robot * delta.block<3, 1>(0, 3);
  motion.tail<3>() = matrixToRotationVector(relative_rotation);
  return motion;
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

  history_.push_back(Sample{relativeMotion(poseToHomogeneous(position, orientation)), 0.0});
  while (history_.size() > limits_.moving_average_samples) {
    history_.pop_front();
  }

  Eigen::Matrix<double, 6, 1> averaged = Eigen::Matrix<double, 6, 1>::Zero();
  for (const auto & entry : history_) {
    averaged += entry.motion;
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

CartesianTwist TeleopIncrementMapper::feedRate(
  const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation, double dt)
{
  CartesianTwist twist;
  if (!active_) {
    return twist;
  }

  // The interval this sample covers: the caller's tick, floored so that a
  // starved loop cannot divide by zero and authorise an unbounded speed.
  const double interval = std::max(1e-3, dt);
  history_.push_back(Sample{relativeMotion(poseToHomogeneous(position, orientation)), interval});
  while (history_.size() > limits_.moving_average_samples) {
    history_.pop_front();
  }

  // The average RATE over the window, which is the total motion over the total
  // time it covered. Averaging the increments and then dividing by one interval
  // would be the same thing only while every sample covered the same interval,
  // and a tick that was late is exactly the case this has to survive.
  //
  // Entries with no interval are entries the increment form pushed - it has no
  // use for one - and are skipped rather than divided by: the two forms are for
  // two configurations of the node, and a session that switched between them
  // without a `start()` would otherwise read a rate out of a zero interval.
  Eigen::Matrix<double, 6, 1> travelled = Eigen::Matrix<double, 6, 1>::Zero();
  double elapsed = 0.0;
  for (const auto & entry : history_) {
    if (entry.dt <= 0.0) {
      continue;
    }
    travelled += entry.motion;
    elapsed += entry.dt;
  }
  if (elapsed <= 0.0) {
    // Nothing has been measured over an interval yet. Reporting no motion is the
    // safe answer: the caller holds.
    return twist;
  }
  const Eigen::Matrix<double, 6, 1> rate = travelled / elapsed;

  const Eigen::Vector3d signs =
    mapping_.rotation_mode == RotationMapping::kTool ?
    mapping_.tool_rotation_signs.cwiseProduct(mapping_.rotation_signs) :
    mapping_.rotation_signs;

  twist.linear = rate.head<3>();
  twist.angular = rate.tail<3>().cwiseProduct(signs);
  twist.valid = true;
  return twist;
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

JointRateLimiter::JointRateLimiter(JointRateLimits limits, std::size_t dof)
: limits_(limits), velocity_(Eigen::VectorXd::Zero(dof)), acceleration_(Eigen::VectorXd::Zero(dof)),
  previous_target_(Eigen::VectorXd::Zero(dof))
{
  // A zero ceiling would freeze the arm and a zero time constant would divide by
  // zero; both are configuration mistakes, and the limiter is on the command
  // path, so they are repaired here rather than trusted.
  limits_.max_velocity = std::abs(limits_.max_velocity);
  limits_.max_acceleration = std::abs(limits_.max_acceleration);
  limits_.max_jerk = std::abs(limits_.max_jerk);
}

Eigen::VectorXd JointRateLimiter::apply(
  const Eigen::VectorXd & target, const Eigen::VectorXd & current, double dt)
{
  if (target.size() != current.size() || target.size() != velocity_.size()) {
    // A target for a different number of joints is a caller bug. Holding the
    // measurement is the only answer that cannot move the wrong joint.
    return current;
  }

  const double safe_dt = std::min(0.05, std::max(1e-3, dt));
  const Eigen::Index count = target.size();

  // Where the target itself is going, per joint. A joint that is merely keeping
  // up with a moving target is not closing on it and needs no room to stop, and
  // separating the two is what lets a steadily moving target be tracked at its
  // own speed with a small gap instead of that gap acting as a brake.
  if (!seeded_ || previous_target_.size() != target.size()) {
    previous_target_ = target;
    seeded_ = true;
  }
  Eigen::VectorXd target_rate = (target - previous_target_) / safe_dt;
  target_rate = target_rate.cwiseMax(-limits_.max_velocity).cwiseMin(limits_.max_velocity);
  previous_target_ = target;

  const double acceleration_step = limits_.max_jerk * safe_dt;
  const Eigen::VectorXd remaining = target - current;

  for (Eigen::Index i = 0; i < count; ++i) {
    const double sign = remaining(i) >= 0.0 ? 1.0 : -1.0;
    const double toward = std::abs(remaining(i));
    const double velocity_toward = sign * velocity_(i);
    // The gap is closed by two motions: the target coming towards the arm, which
    // costs nothing to follow and needs no room to stop from, and the arm
    // closing on it, which does.
    const double rate_toward = std::max(0.0, sign * target_rate(i));

    // The speed the arm should have: the target's own rate, plus as much closing
    // speed as it can still be stopped from inside the distance left.
    //
    // The stopping law is the jerk aware one - v^2 / (2 a) for the stop, plus
    // v a / (2 j) for the distance covered while the deceleration is built - and
    // what is wanted is the speed the arm may hold while still being able to
    // stop, not the speed it could stop from this instant. Those are different
    // questions, and answering the second one is what makes a limiter brake too
    // late: both the loop that follows the curve and the jerk ramp ahead of it
    // are paid for out of the same distance, so the distance they need grows
    // with the speed they allow. Substituting that margin into the law gives a
    // quadratic in v, which is solved here rather than iterated:
    //
    //   v = 0.5 * ( -k + sqrt( k^2 + 8 a s ) ),  k = a^2 / j + 2 a m
    //
    // with `m` the time the loop and a full acceleration ramp take. The floor
    // keeps the last fraction of a millimetre closing instead of being
    // approached for ever.
    const double stopping_law = limits_.max_acceleration * limits_.max_acceleration /
      std::max(1e-9, limits_.max_jerk);
    const double margin_seconds = 1.0 / speed_gain_ +
      2.0 * limits_.max_acceleration / std::max(1e-9, limits_.max_jerk);
    const double quadratic = stopping_law + 2.0 * limits_.max_acceleration * margin_seconds;
    const double closing_limit = std::min(
      limits_.max_velocity,
      std::max(
        0.01 * limits_.max_velocity,
        0.5 * (-quadratic + std::sqrt(quadratic * quadratic +
        8.0 * limits_.max_acceleration * toward))));
    const double speed_limit = std::min(limits_.max_velocity, rate_toward + closing_limit);

    // Chase that speed with a first order loop, then put the result through the
    // two limits that make the command stream smooth: the speed may not change
    // faster than max_acceleration, and the acceleration may not change faster
    // than max_jerk. The second is what makes the speed curve an S rather than a
    // ramp, so a joint starts and stops softly instead of asking the firmware
    // for a step in speed.
    const double wanted_toward = std::max(
      -limits_.max_acceleration,
      std::min(limits_.max_acceleration, speed_gain_ * (speed_limit - velocity_toward)));
    // The speed limit is a distance, so it is worked out in the frame of the
    // distance: "towards the target" is always the positive direction there. The
    // acceleration has to be turned back into the joint's own frame before it is
    // applied, or the limiter only ever moves one way - a joint whose target is
    // below it would be told to accelerate upwards, the guard below would refuse
    // the step, and the arm would sit still while every target on the other side
    // of it was ignored.
    const double wanted_acceleration = sign * wanted_toward;
    acceleration_(i) = std::max(
      acceleration_(i) - acceleration_step,
      std::min(acceleration_(i) + acceleration_step, wanted_acceleration));
    velocity_(i) = std::max(
      -limits_.max_velocity, std::min(limits_.max_velocity, velocity_(i) + acceleration_(i) * safe_dt));
  }

  // The final guard: never step past the target, in either direction. The speed
  // limit above starts the stop early enough that what this trims is a fraction
  // of one cycle's step rather than a whole one - a limiter that is behind has
  // to be able to stop where it is asked to, and one that overshoots turns into
  // an oscillator around the target. The velocity state is left alone: forcing
  // it to what the guard allowed would be a step in speed in its own right,
  // which is the thing this class exists to remove.
  Eigen::VectorXd step = velocity_ * safe_dt;
  for (Eigen::Index i = 0; i < count; ++i) {
    if (remaining(i) >= 0.0) {
      step(i) = std::max(0.0, std::min(step(i), remaining(i)));
    } else {
      step(i) = std::min(0.0, std::max(step(i), remaining(i)));
    }
  }
  return current + step;
}

void JointRateLimiter::reset()
{
  velocity_.setZero();
  acceleration_.setZero();
  seeded_ = false;
}

JointVelocityProfile::JointVelocityProfile(JointVelocityLimits limits, std::size_t dof)
: limits_(limits), velocity_(Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dof)))
{
  // The same reading as every other limit in this file: a zero ceiling would
  // freeze the arm and a negative time constant would smooth backwards.
  limits_.max_velocity = std::abs(limits_.max_velocity);
  limits_.max_acceleration = std::abs(limits_.max_acceleration);
  limits_.filter_tau = std::abs(limits_.filter_tau);

  // The state starts at rest rather than at the first command, which is what
  // Servo's filter reset does as well (it seeds from the joint state, not from
  // the command). Seeding from the command would put a step of speed in the
  // field the firmware reads as a slew ceiling on the very first cycle of a
  // following session, which is the one moment every other part of this stack
  // is careful to start softly.
}

Eigen::VectorXd JointVelocityProfile::apply(const Eigen::VectorXd & target, double dt)
{
  if (target.size() != velocity_.size()) {
    // A command for a different number of joints is a caller bug. Holding the
    // last speed would move the wrong axes, so the arm is held still instead.
    velocity_.setZero();
    return velocity_;
  }

  const double safe_dt = sanitiseDeltaSeconds(dt);

  // Filter first, then the ceilings, which is Servo's order as well: limiting
  // before smoothing would let the filter ramp towards a speed the limit had
  // already refused, and the filter's own state would carry it there.
  const double alpha = safe_dt / (safe_dt + limits_.filter_tau);
  const double acceleration_step = limits_.max_acceleration * safe_dt;
  for (Eigen::Index i = 0; i < velocity_.size(); ++i) {
    const double smoothed = velocity_(i) + alpha * (target(i) - velocity_(i));
    const double bounded = std::max(
      velocity_(i) - acceleration_step, std::min(velocity_(i) + acceleration_step, smoothed));
    velocity_(i) = std::max(-limits_.max_velocity, std::min(limits_.max_velocity, bounded));
  }
  return velocity_;
}

void JointVelocityProfile::reset()
{
  velocity_.setZero();
}

Eigen::VectorXd integrateJointVelocity(
  const Eigen::VectorXd & position, const Eigen::VectorXd & velocity, double dt,
  const Eigen::VectorXd & lower, const Eigen::VectorXd & upper)
{
  if (position.size() != velocity.size()) {
    // Nothing can be integrated against a speed that belongs to other joints.
    // Staying where the reference is beats moving it by a guess.
    return position;
  }

  Eigen::VectorXd next = position + velocity * sanitiseDeltaSeconds(dt);
  if (lower.size() == next.size() && upper.size() == next.size()) {
    next = next.cwiseMax(lower).cwiseMin(upper);
  }
  return next;
}

double leadScale(const Eigen::VectorXd & error, double margin)
{
  if (error.size() == 0 || margin <= 0.0) {
    return 1.0;
  }
  const double worst = error.cwiseAbs().maxCoeff();
  // Inside the soft band the command is untouched, because a lead smaller than
  // this is just the arm's tracking error and scaling for it would make the arm
  // slower than the operator every time it lagged at all.
  const double soft = 0.5 * margin;
  if (worst <= soft) {
    return 1.0;
  }
  if (worst >= margin) {
    return 0.0;
  }
  return (margin - worst) / (margin - soft);
}

CartesianTwist twistAtOffset(const CartesianTwist & twist, const Eigen::Vector3d & offset)
{
  CartesianTwist carried = twist;
  carried.linear = twist.linear - twist.angular.cross(offset);
  return carried;
}

}  // namespace litearm_vr_teleop
