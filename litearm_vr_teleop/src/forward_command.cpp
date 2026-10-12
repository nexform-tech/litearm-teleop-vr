// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.

#include "litearm_vr_teleop/forward_command.hpp"

#include <algorithm>
#include <cmath>

namespace litearm_vr_teleop
{

ForwardCommandTracker::ForwardCommandTracker(ForwardCommandLimits limits)
: limits_(limits), positions_(Eigen::VectorXd::Zero(limits.dof)),
  velocities_(Eigen::VectorXd::Zero(limits.dof)), seeded_(false)
{
}

void ForwardCommandTracker::reset(const Eigen::VectorXd & measured)
{
  positions_ = measured;
  velocities_.setZero(positions_.size());
  seeded_ = true;
}

bool ForwardCommandTracker::update(
  const Eigen::VectorXd * target, const Eigen::VectorXd & measured, double command_age,
  bool new_command, double dt, Eigen::VectorXd & positions, Eigen::VectorXd & velocities)
{
  if (positions.size() != positions_.size() || velocities.size() != positions_.size()) {
    // The caller's buffers are not sized for this joint count. Writing anything
    // would be out of bounds, and the caller has a bug it should see rather than
    // a hold it could mistake for normal operation.
    return true;
  }

  const bool stale = limits_.command_timeout_s > 0.0 && command_age > limits_.command_timeout_s;
  const bool usable =
    target != nullptr && target->size() == positions_.size() && command_age >= 0.0 && !stale;

  if (!usable) {
    // Hold. The velocity is written as zero deliberately: this arm's only way to
    // stay put is a zero slew ceiling with the reference left where it was, so a
    // hold has to be stated rather than implied by silence.
    if (measured.size() == positions_.size()) {
      positions_ = measured;
    }
    positions = positions_;
    velocities.setZero();
    return true;
  }

  if (!seeded_) {
    // No measured pose was ever supplied. Accept the first setpoint as the
    // starting point and command no motion from it, rather than derive a
    // velocity from a distance to an unknown origin.
    positions_ = *target;
    velocities_.setZero(positions_.size());
    seeded_ = true;
    positions = positions_;
    velocities.setZero();
    return false;
  }

  if (new_command) {
    // The one cycle per setpoint where there is something to differentiate. The
    // previous positions are still intact while the velocity is computed from
    // them, and are only overwritten afterwards.
    const double inverse_dt = 1.0 / std::max(1e-3, dt);
    const Eigen::Index count = positions_.size();
    positions = *target;
    for (Eigen::Index i = 0; i < count; ++i) {
      double velocity = (positions(i) - positions_(i)) * inverse_dt;
      if (limits_.max_velocity > 0.0) {
        velocity = std::clamp(velocity, -limits_.max_velocity, limits_.max_velocity);
      }
      velocities_(i) = velocity;
    }
    positions_ = positions;
  } else {
    // Nothing arrived: the setpoint is the one from last cycle and the ceiling
    // it needs is the one worked out for it then.
    positions = *target;
  }

  // Coefficient-wise, into the caller's buffers: this runs every control cycle,
  // so no expression is allowed to reach the heap.
  velocities = velocities_;
  return false;
}

}  // namespace litearm_vr_teleop
