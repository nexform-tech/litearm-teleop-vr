// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.

#include "litearm_vr_teleop/joint_move.hpp"

#include <algorithm>
#include <cmath>

namespace litearm_vr_teleop
{
namespace
{

/// Cycle time band, matching the incremental mapping. A descheduled process
/// must not be allowed to authorise a correspondingly large step on resume, and
/// a zero dt must not divide the step limit to zero.
double sanitiseDeltaSeconds(double dt)
{
  return std::min(0.05, std::max(1e-3, dt));
}

/// Floor on the rate limits so that a zero (or negative) configuration value
/// cannot turn the profile into a division by zero or a joint that never moves.
constexpr double kMinimumRateLimit = 1e-4;

/// Bring a rate limit vector to `dof` entries, taking the magnitude of each and
/// flooring it. A short or empty vector is filled with `fallback` rather than
/// left short, because indexing past the end of a rate limit is the kind of bug
/// that shows up as a joint moving at full speed.
Eigen::VectorXd normaliseRateLimits(
  const Eigen::VectorXd & values, Eigen::Index dof, double fallback)
{
  Eigen::VectorXd result = Eigen::VectorXd::Constant(dof, fallback);
  const Eigen::Index shared = std::min(dof, values.size());
  for (Eigen::Index i = 0; i < shared; ++i) {
    result(i) = std::max(kMinimumRateLimit, std::abs(values(i)));
  }
  return result;
}

}  // namespace

double JointMoveProfile::travelEpsilon() const
{
  // With a velocity ceiling of 2 rad/s and a 5 ms cycle, the smallest motion a
  // cycle can express is about 1e-4 rad. Anything below this is arrival, not
  // motion; comparing against it is what lets a joint stop instead of
  // asymptotically approaching the target forever.
  return 1e-5;
}

JointMoveProfile::JointMoveProfile(
  const Eigen::VectorXd & start, const Eigen::VectorXd & target,
  const Eigen::VectorXd & max_speed, const Eigen::VectorXd & max_accel,
  const Eigen::VectorXd & lower_limits, const Eigen::VectorXd & upper_limits)
: target_(target),
  command_(start),
  velocity_(Eigen::VectorXd::Zero(start.size())),
  max_speed_(normaliseRateLimits(max_speed, start.size(), 1.0)),
  max_accel_(normaliseRateLimits(max_accel, start.size(), 1.0)),
  initial_travel_(Eigen::VectorXd::Zero(start.size()))
{
  if (lower_limits.size() == target_.size() && upper_limits.size() == target_.size()) {
    target_ = target_.cwiseMax(lower_limits).cwiseMin(upper_limits);
  }

  for (Eigen::Index i = 0; i < target_.size(); ++i) {
    initial_travel_(i) = std::abs(target_(i) - command_(i));
  }
}

JointMoveProfile::Point JointMoveProfile::step(double dt)
{
  const double safe_dt = sanitiseDeltaSeconds(dt);
  const double epsilon = travelEpsilon();

  for (Eigen::Index i = 0; i < command_.size(); ++i) {
    const double speed_limit = max_speed_(i);
    const double accel_limit = max_accel_(i);
    const double delta_speed_max = accel_limit * safe_dt;

    const double remaining = target_(i) - command_(i);
    double speed = velocity_(i);

    if (std::abs(remaining) < epsilon && std::abs(speed) < delta_speed_max) {
      // Close enough and slow enough: snap and stop, so the profile reaches a
      // definite end instead of an asymptote.
      command_(i) = target_(i);
      velocity_(i) = 0.0;
      continue;
    }

    // Distance needed to come to rest at the acceleration limit. Once the
    // remaining travel is within it, the only way to arrive without overshoot is
    // to stop accelerating toward the target and start decelerating.
    const double stopping_distance = (speed * speed) / (2.0 * accel_limit);
    const bool decelerating = (remaining * speed > 0.0) && (std::abs(remaining) <= stopping_distance);
    const double desired_speed = decelerating ? 0.0 : std::copysign(speed_limit, remaining);

    const double delta_speed = std::clamp(desired_speed - speed, -delta_speed_max, delta_speed_max);
    speed = std::clamp(speed + delta_speed, -speed_limit, speed_limit);

    const double increment = speed * safe_dt;
    if (remaining * increment > 0.0 && std::abs(increment) >= std::abs(remaining)) {
      // This cycle's motion covers the rest of the travel; arrive exactly rather
      // than stepping over the target and needing a reversal to come back.
      command_(i) = target_(i);
      velocity_(i) = 0.0;
    } else {
      command_(i) += increment;
      velocity_(i) = speed;
    }
  }

  done_ = true;
  for (Eigen::Index i = 0; i < command_.size(); ++i) {
    if (std::abs(target_(i) - command_(i)) >= epsilon || std::abs(velocity_(i)) >= epsilon) {
      done_ = false;
      break;
    }
  }

  if (done_) {
    // Arrival is made exact rather than approximate. The alternative is a
    // profile that is "done" while still a few times 1e-5 rad away from its
    // target, which every caller then has to know about. Nothing here cares
    // about the difference; callers care a great deal about being able to test
    // for arrival.
    command_ = target_;
    velocity_.setZero();
  }

  Point point;
  point.position = command_;
  point.velocity = velocity_;
  point.done = done_;
  return point;
}

double JointMoveProfile::progress() const
{
  double worst = 1.0;
  for (Eigen::Index i = 0; i < command_.size(); ++i) {
    if (initial_travel_(i) < travelEpsilon()) {
      continue;
    }
    worst = std::min(worst, 1.0 - std::abs(target_(i) - command_(i)) / initial_travel_(i));
  }
  return std::clamp(worst, 0.0, 1.0);
}

}  // namespace litearm_vr_teleop
