// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.
//
// The teleoperation start pose, in one place.
//
// Two nodes need these numbers: vr_teleop_node moves the arm to this pose when
// the operator presses A, and vr_target_viz runs forward kinematics on it to
// anchor the pose it draws in RViz. The viewer is the reason the constant lives
// in a header rather than in the node's anonymous namespace where it started: a
// second copy would be free to drift, and the symptom of a drifted copy is a
// target drawn where the arm is not being sent, which reads like a mapping bug
// rather than like a stale constant. It has to be the same numbers, so it is the
// same symbol.
//
// The values are the `grasp_ready` values of the SRDF in
// litearm_vr_moveit_config, and the two copies have to move together: the SRDF
// makes the pose nameable for MoveIt, this one makes it reachable without a
// planner. If they drift, a teleoperation session starts somewhere other than
// where a planning session does, which is the kind of difference nobody thinks
// to check. At run time `start_pose` in config/vr_teleop.yaml overrides this
// fallback, and test/test_kinematics_consistency.py is what keeps that file
// honest against the SRDF.

#ifndef LITEARM_VR_TELEOP__TELEOP_START_POSE_HPP_
#define LITEARM_VR_TELEOP__TELEOP_START_POSE_HPP_

namespace litearm_vr_teleop
{

/// The teleoperation start pose, in radians, in joint order (joint1..joint7).
///
/// `inline` so that every translation unit that includes this header shares the
/// one definition, which is what C++17 is for; nothing here needs a symbol in a
/// library.
inline constexpr double kTeleopStartPose[7] = {
  0.0, -0.645772, 0.0, -1.989675, 0.0, -0.837758, 0.0,
};

}  // namespace litearm_vr_teleop

#endif  // LITEARM_VR_TELEOP__TELEOP_START_POSE_HPP_
