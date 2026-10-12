// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.
//
// When the inverse kinematics has to be damped, as a rule with no ROS in it.
//
// It lives in its own header rather than next to the planner because the test
// that pins it down is a pure logic test: pulling in rclcpp to check a
// comparison would make the one thing a reviewer can run without a robot model
// depend on a robot model.

#ifndef LITEARM_VR_TELEOP__CONDITIONING_GUARD_HPP_
#define LITEARM_VR_TELEOP__CONDITIONING_GUARD_HPP_

namespace litearm_vr_teleop
{

/// Whether a Jacobian's conditioning calls for the damped solver.
///
/// `threshold` of zero or less turns the test off, which is what a workspace
/// with no damping to fall back on wants. The comparison is strict, so a
/// threshold of 0.05 means "below 0.05" and not "at or below it".
inline bool conditioningNeedsDamping(double minimum_singular_value, double threshold)
{
  return threshold > 0.0 && minimum_singular_value < threshold;
}

}  // namespace litearm_vr_teleop

#endif  // LITEARM_VR_TELEOP__CONDITIONING_GUARD_HPP_
