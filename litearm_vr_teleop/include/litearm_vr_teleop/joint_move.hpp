// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.
//
// Point to point move in joint space, rate limited, joints independent.

#ifndef LITEARM_VR_TELEOP__JOINT_MOVE_HPP_
#define LITEARM_VR_TELEOP__JOINT_MOVE_HPP_

#include <cstddef>

#include <Eigen/Core>

namespace litearm_vr_teleop
{

/// Rate limited point to point move in joint space.
///
/// The semantics deliberately follow the arm SDK (`litearm-cpp`'s
/// `Arm::movej`, firmware command CMD_MOVE_J): **every joint travels at the same
/// speed and therefore arrives whenever it arrives**. A joint with a small
/// travel arrives early and waits, a joint with a large travel sets the
/// duration. That is `movej`. It is explicitly not `movej_sync`, which scales a
/// single path parameter so that all joints arrive together and which the SDK
/// offers as a separate mode for that reason.
///
/// This is implemented here rather than delegated to the controller manager's
/// joint trajectory controller because the reference behaviour is a per joint
/// rate limit with a stopping-distance test, not a time-parameterised
/// trajectory. The reference follow server in vr_control states the same law in
/// `_slew_target`: accelerate toward the target at the joint's acceleration
/// limit, cruise at its speed limit, and begin decelerating once the remaining
/// travel equals v^2 / (2 a). Keeping the same law means a teleoperation start
/// produced here looks like the one that layer produced, which matters when
/// comparing the two stacks.
///
/// The profile is stateful. `step()` is called once per control cycle and the
/// caller owns the cycle time.
class JointMoveProfile
{
public:
  /// One cycle of the profile.
  struct Point
  {
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    /// True once every joint has reached its target.
    bool done = false;
  };

  /// `start` is where the arm is now, `target` where it should end up.
  ///
  /// `lower_limits` and `upper_limits` bound the target: a move to a pose
  /// outside the joint range would otherwise be commanded in full and refused by
  /// the hardware, or worse, accepted on a mock. Both are optional, and passing
  /// empty vectors means "no limits" - which is NOT the same as passing `start`,
  /// a mistake worth naming because it silently clamps the target to where the
  /// arm already is and produces a profile that is instantly done. `start` is
  /// never clamped, on the assumption that it is a measured value and therefore
  /// already inside.
  JointMoveProfile(
    const Eigen::VectorXd & start, const Eigen::VectorXd & target,
    const Eigen::VectorXd & max_speed, const Eigen::VectorXd & max_accel,
    const Eigen::VectorXd & lower_limits, const Eigen::VectorXd & upper_limits);

  /// Advance the profile by `dt` seconds and return the new command.
  Point step(double dt);

  bool done() const { return done_; }

  /// Current command, the last position produced by `step()`.
  const Eigen::VectorXd & position() const { return command_; }

  /// Progress of the slowest joint, in [0, 1].
  ///
  /// With independent joint arrival there is no single completion fraction, so
  /// this reports the worst case: how far the joint with the most remaining
  /// travel still has to go. It is a diagnostic figure for logging, not a
  /// control input.
  double progress() const;

private:
  double travelEpsilon() const;

  Eigen::VectorXd target_;
  Eigen::VectorXd command_;
  Eigen::VectorXd velocity_;
  Eigen::VectorXd max_speed_;
  Eigen::VectorXd max_accel_;
  /// Initial distance each joint had to cover, for `progress()` only.
  Eigen::VectorXd initial_travel_;
  bool done_ = false;
};

}  // namespace litearm_vr_teleop

#endif  // LITEARM_VR_TELEOP__JOINT_MOVE_HPP_
