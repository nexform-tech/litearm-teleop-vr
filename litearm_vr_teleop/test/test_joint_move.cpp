// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.
//
// Tests for the point to point profile.
//
// The property under test is the one that distinguishes this from a
// time-parameterised trajectory: every joint travels at the same speed and
// therefore ARRIVES WHENEVER IT ARRIVES. A joint with a short travel finishes
// early and stops; it does not slow down so that all joints land together, which
// is what the SDK's movej_sync mode does and what this deliberately is not.

#include <cmath>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "litearm_vr_teleop/joint_move.hpp"

namespace litearm_vr_teleop
{
namespace
{

constexpr double kCycleSeconds = 0.01;

/// Passing empty limits means "no limits". Passing `start` as both limits looks
/// like a way of saying the same thing and is not: it clamps the target to where
/// the arm already is, which produces a profile that is instantly done and hides
/// whatever the test was actually about.
const Eigen::VectorXd kNoLimits;

/// Run a profile to completion, returning the number of cycles it took.
/// Bounded so a profile that never finishes fails the test instead of hanging.
int runToCompletion(JointMoveProfile & profile, int max_cycles = 20000)
{
  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    if (profile.step(kCycleSeconds).done) {
      return cycle + 1;
    }
  }
  return -1;
}

TEST(JointMove, ReachesTheTargetExactlyAndStops)
{
  const Eigen::VectorXd start = (Eigen::VectorXd(2) << 0.0, 0.0).finished();
  const Eigen::VectorXd target = (Eigen::VectorXd(2) << 0.5, -0.3).finished();
  const Eigen::VectorXd speed = (Eigen::VectorXd(2) << 0.5, 0.5).finished();
  const Eigen::VectorXd accel = (Eigen::VectorXd(2) << 3.0, 3.0).finished();

  JointMoveProfile profile(start, target, speed, accel, kNoLimits, kNoLimits);
  const int cycles = runToCompletion(profile);

  EXPECT_GT(cycles, 0);
  EXPECT_TRUE(profile.done());
  EXPECT_TRUE(profile.position().isApprox(target, 1e-12));
}

TEST(JointMove, JointsArriveAtDifferentTimesAtTheSameSpeed)
{
  // Joint 0 has four times the travel of joint 1. If both moved at the same
  // speed, joint 1 arrives after roughly a quarter of the time.
  const Eigen::VectorXd start = (Eigen::VectorXd(2) << 0.0, 0.0).finished();
  const Eigen::VectorXd target = (Eigen::VectorXd(2) << 1.0, 0.25).finished();
  const Eigen::VectorXd speed = (Eigen::VectorXd(2) << 0.5, 0.5).finished();
  const Eigen::VectorXd accel = (Eigen::VectorXd(2) << 3.0, 3.0).finished();

  JointMoveProfile profile(start, target, speed, accel, kNoLimits, kNoLimits);

  double short_joint_arrival = -1.0;
  int cycles = 0;
  while (cycles < 20000) {
    const JointMoveProfile::Point point = profile.step(kCycleSeconds);
    ++cycles;
    if (short_joint_arrival < 0.0 &&
      std::abs(point.position(1) - target(1)) < 1e-9 && std::abs(point.velocity(1)) < 1e-9)
    {
      short_joint_arrival = cycles * kCycleSeconds;
    }
    if (point.done) {
      break;
    }
  }

  ASSERT_GT(short_joint_arrival, 0.0) << "the short joint never reported arrival";
  const double long_joint_arrival = cycles * kCycleSeconds;
  // Between a quarter and a half of the long joint's time: a quarter would be
  // pure constant speed, the extra comes from both joints sharing the same
  // acceleration ramp.
  EXPECT_GT(long_joint_arrival / short_joint_arrival, 2.5);
  EXPECT_LT(long_joint_arrival / short_joint_arrival, 4.5);
}

TEST(JointMove, NeverExceedsTheSpeedLimit)
{
  const Eigen::VectorXd start = (Eigen::VectorXd(1) << 0.0).finished();
  const Eigen::VectorXd target = (Eigen::VectorXd(1) << 2.0).finished();
  const Eigen::VectorXd speed = (Eigen::VectorXd(1) << 0.5).finished();
  const Eigen::VectorXd accel = (Eigen::VectorXd(1) << 3.0).finished();

  JointMoveProfile profile(start, target, speed, accel, kNoLimits, kNoLimits);

  double previous = 0.0;
  for (int cycle = 0; cycle < 20000; ++cycle) {
    const JointMoveProfile::Point point = profile.step(kCycleSeconds);
    EXPECT_LE(std::abs(point.velocity(0)), 0.5 + 1e-9);
    // The step actually taken must agree with the velocity the profile reports,
    // otherwise the profile and the motion have drifted apart and the reported
    // velocity is a lie that any downstream consumer would believe.
    EXPECT_NEAR((point.position(0) - previous) / kCycleSeconds, point.velocity(0), 1e-6);
    previous = point.position(0);
    if (point.done) {
      break;
    }
  }
}

TEST(JointMove, ClampsTheTargetIntoTheJointRange)
{
  const Eigen::VectorXd start = (Eigen::VectorXd(1) << 0.0).finished();
  const Eigen::VectorXd target = (Eigen::VectorXd(1) << 5.0).finished();
  const Eigen::VectorXd speed = (Eigen::VectorXd(1) << 0.5).finished();
  const Eigen::VectorXd accel = (Eigen::VectorXd(1) << 3.0).finished();
  const Eigen::VectorXd lower = (Eigen::VectorXd(1) << -1.0).finished();
  const Eigen::VectorXd upper = (Eigen::VectorXd(1) << 1.2).finished();

  JointMoveProfile profile(start, target, speed, accel, lower, upper);
  runToCompletion(profile);

  // The move stops at the joint limit. Commanding the requested 5.0 would be
  // refused by the hardware at best, and on a mock it would look like it worked.
  EXPECT_NEAR(profile.position()(0), 1.2, 1e-12);
}

TEST(JointMove, AlreadyThereFinishesImmediately)
{
  const Eigen::VectorXd start = (Eigen::VectorXd(2) << 0.3, -0.4).finished();
  const Eigen::VectorXd speed = (Eigen::VectorXd(2) << 0.5, 0.5).finished();
  const Eigen::VectorXd accel = (Eigen::VectorXd(2) << 3.0, 3.0).finished();

  JointMoveProfile profile(start, start, speed, accel, kNoLimits, kNoLimits);
  EXPECT_TRUE(profile.step(kCycleSeconds).done);
  EXPECT_NEAR(profile.progress(), 1.0, 1e-12);
}

}  // namespace
}  // namespace litearm_vr_teleop
