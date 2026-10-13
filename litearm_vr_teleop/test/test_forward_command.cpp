// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.
//
// Tests for the forward controller's arithmetic.
//
// The behaviour under test is the part that is easy to get wrong and hard to
// notice: on this arm a position written without a velocity freezes the joint
// reference, so the controller's failure mode is not a wrong motion but no
// motion at all, with nothing reported anywhere.

#include <cmath>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "litearm_vr_teleop/forward_command.hpp"

namespace litearm_vr_teleop
{
namespace
{

constexpr double kCycleSeconds = 0.01;

ForwardCommandTracker makeTracker(double max_velocity = 1.0, double timeout = 0.2)
{
  return ForwardCommandTracker(ForwardCommandLimits{3, max_velocity, timeout});
}

TEST(ForwardCommand, HoldsAtTheMeasuredPoseBeforeAnyCommand)
{
  ForwardCommandTracker tracker = makeTracker();
  const Eigen::VectorXd measured = (Eigen::VectorXd(3) << 0.1, 0.2, 0.3).finished();
  tracker.reset(measured);

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(3);
  const bool holding = tracker.update(nullptr, measured, -1.0, true, kCycleSeconds, positions, velocities);

  EXPECT_TRUE(holding);
  // The hold is at the measurement, and the velocity is written as zero because
  // that is this firmware's only way to stand still.
  EXPECT_TRUE(positions.isApprox(measured));
  EXPECT_TRUE(velocities.isZero(0.0));
}

TEST(ForwardCommand, DerivesVelocityFromTheSetpointStream)
{
  ForwardCommandTracker tracker = makeTracker(1.0);
  const Eigen::VectorXd measured = Eigen::VectorXd::Zero(3);
  tracker.reset(measured);

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(3);

  // A setpoint 0.004 rad away after 10 ms is 0.4 rad/s, inside the ceiling.
  const Eigen::VectorXd target = (Eigen::VectorXd(3) << 0.004, 0.0, 0.0).finished();
  const bool holding =
    tracker.update(&target, measured, 0.0, true, kCycleSeconds, positions, velocities);

  EXPECT_FALSE(holding);
  EXPECT_TRUE(positions.isApprox(target));
  EXPECT_NEAR(velocities(0), 0.4, 1e-9);
  EXPECT_NEAR(velocities(1), 0.0, 1e-12);
}

TEST(ForwardCommand, ClampsTheVelocityCeiling)
{
  ForwardCommandTracker tracker = makeTracker(0.5);
  const Eigen::VectorXd measured = Eigen::VectorXd::Zero(1);
  tracker.reset(measured);

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(1);

  // A large jump would imply 10 rad/s; the ceiling must hold it at 0.5. Note the
  // position still goes to the setpoint: on this firmware the velocity is the
  // slew ceiling of the reference, not a filter on the command.
  const Eigen::VectorXd target = (Eigen::VectorXd(1) << 0.1).finished();
  tracker.update(&target, measured, 0.0, true, kCycleSeconds, positions, velocities);

  EXPECT_NEAR(positions(0), 0.1, 1e-12);
  EXPECT_NEAR(velocities(0), 0.5, 1e-12);
}

TEST(ForwardCommand, GoesStaleAndHolds)
{
  ForwardCommandTracker tracker = makeTracker(1.0, 0.2);
  const Eigen::VectorXd measured = Eigen::VectorXd::Zero(1);
  tracker.reset(measured);

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(1);

  const Eigen::VectorXd target = (Eigen::VectorXd(1) << 0.05).finished();
  tracker.update(&target, measured, 0.0, true, kCycleSeconds, positions, velocities);
  EXPECT_NEAR(positions(0), 0.05, 1e-12);

  // The publisher died. Past the timeout the output must be a hold, and the
  // velocity must be zero rather than the last derived one, because a nonzero
  // velocity with a stale position is exactly how an arm walks away.
  const Eigen::VectorXd now_measured = (Eigen::VectorXd(1) << 0.048).finished();
  const bool holding =
    tracker.update(&target, now_measured, 0.25, true, kCycleSeconds, positions, velocities);

  EXPECT_TRUE(holding);
  EXPECT_TRUE(positions.isApprox(now_measured));
  EXPECT_DOUBLE_EQ(velocities(0), 0.0);
}

TEST(ForwardCommand, RejectsAWrongSizedSetpoint)
{
  ForwardCommandTracker tracker = makeTracker();
  const Eigen::VectorXd measured = Eigen::VectorXd::Zero(3);
  tracker.reset(measured);

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(3);

  const Eigen::VectorXd wrong_size = Eigen::VectorXd::Zero(2);
  const bool holding =
    tracker.update(&wrong_size, measured, 0.0, true, kCycleSeconds, positions, velocities);

  EXPECT_TRUE(holding);
  EXPECT_TRUE(positions.isApprox(measured));
}

TEST(ForwardCommand, ZeroTimeoutDisablesTheStalenessTest)
{
  ForwardCommandTracker tracker = makeTracker(1.0, 0.0);
  const Eigen::VectorXd measured = Eigen::VectorXd::Zero(1);
  tracker.reset(measured);

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(1);

  const Eigen::VectorXd target = (Eigen::VectorXd(1) << 0.01).finished();
  const bool holding =
    tracker.update(&target, measured, 1000.0, true, kCycleSeconds, positions, velocities);

  // A very old command is still followed when the test is switched off. That is
  // the documented consequence of setting the timeout to zero.
  EXPECT_FALSE(holding);
  EXPECT_TRUE(positions.isApprox(target));
}

/// The velocity belongs to the operator's motion, not to the control loop.
///
/// The same 0.02 rad step means 0.46 rad/s if the commands are 43 ms apart - the
/// rate the teleoperation node actually achieves while it solves inverse
/// kinematics - and 2.0 rad/s if the step is divided by the controller's own
/// 10 ms cycle, which the ceiling then clips to 1.0. The second is what made
/// every step ask for full speed; this test is the difference between them.
TEST(ForwardCommand, VelocityIsTheStepOverTheIntervalItHappenedOver)
{
  const Eigen::VectorXd measured = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd target = (Eigen::VectorXd(1) << 0.02).finished();
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(1);

  ForwardCommandTracker slow_stream = makeTracker(1.0, 0.2);
  slow_stream.reset(measured);
  slow_stream.update(&target, measured, 0.0, true, 0.043, positions, velocities);
  EXPECT_NEAR(velocities(0), 0.02 / 0.043, 1e-9);
  EXPECT_LT(velocities(0), 1.0);

  ForwardCommandTracker fast_stream = makeTracker(1.0, 0.2);
  fast_stream.reset(measured);
  fast_stream.update(&target, measured, 0.0, true, kCycleSeconds, positions, velocities);
  EXPECT_NEAR(velocities(0), 1.0, 1e-9);  // 2.0 rad/s, clipped by the ceiling
}

/// The velocity is a property of the step, and the step belongs to one cycle.
///
/// This is the difference between a slew ceiling and a sawtooth. The controller
/// runs at 100 Hz and the command stream does not, so most cycles have no arrival
/// in them; recomputing the velocity on those cycles puts zero in the field, and
/// the firmware advances the position reference at the ceiling it is given. A
/// field that is zero half the time is therefore an arm that moves half as far as
/// it was told to, and a velocity plot of a teleoperation session shows a band of
/// spikes where it should show a curve.
TEST(ForwardCommand, HoldsTheVelocityBetweenCommandArrivals)
{
  const Eigen::VectorXd measured = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd target = (Eigen::VectorXd(1) << 0.02).finished();
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(1);

  ForwardCommandTracker tracker = makeTracker(1.0, 0.2);
  tracker.reset(measured);
  tracker.update(&target, measured, 0.0, true, 0.02, positions, velocities);
  const double requested = velocities(0);
  EXPECT_NEAR(requested, 1.0, 1e-9);   // 0.02 rad over 20 ms is the ceiling

  // Four cycles with nothing new arriving: same setpoint, same ceiling, no zero.
  for (int cycle = 0; cycle < 4; ++cycle) {
    positions.setZero();
    velocities.setZero();
    const bool holding = tracker.update(
      &target, measured, kCycleSeconds, false, kCycleSeconds, positions, velocities);
    EXPECT_FALSE(holding);
    EXPECT_DOUBLE_EQ(velocities(0), requested);
    EXPECT_DOUBLE_EQ(positions(0), target(0));
  }
}

/// A sender that knows the speed it is asking for has it written verbatim.
///
/// This is the difference between a ceiling and a guess at one. The teleoperation
/// node runs the rate limiter, so the speed it wants the reference slewed at is a
/// number it already has; deriving it here instead would replace that number with
/// the difference of two commands over the interval between their arrivals, which
/// is measured on this loop's clock and carries the arrival jitter.
TEST(ForwardCommand, WritesTheVelocityTheSenderStated)
{
  ForwardCommandTracker tracker = makeTracker(1.0, 0.2);
  const Eigen::VectorXd measured = Eigen::VectorXd::Zero(2);
  tracker.reset(measured);

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(2);

  // The step would derive to 10 rad/s (0.1 rad in one 10 ms cycle) and the
  // stated figure is well under the ceiling, so nothing here agrees by accident.
  const Eigen::VectorXd target = (Eigen::VectorXd(2) << 0.1, -0.05).finished();
  const Eigen::VectorXd stated = (Eigen::VectorXd(2) << 0.3, -0.2).finished();
  const bool holding =
    tracker.update(&target, &stated, measured, 0.0, true, kCycleSeconds, positions, velocities);

  EXPECT_FALSE(holding);
  EXPECT_TRUE(positions.isApprox(target));
  EXPECT_NEAR(velocities(0), 0.3, 1e-12);
  EXPECT_NEAR(velocities(1), -0.2, 1e-12);
}

/// A stated velocity is a request, and the ceiling still applies to it.
///
/// The sender's figure is what the reference should be slewed at, so a sender
/// asking for more than this controller allows gets the smaller of the two: the
/// firmware's speed limit is a fault rather than a slow-down, and the limit here
/// exists so a bug upstream cannot reach it.
TEST(ForwardCommand, ClampsAStatedVelocityToTheCeiling)
{
  ForwardCommandTracker tracker = makeTracker(0.5, 0.2);
  const Eigen::VectorXd measured = Eigen::VectorXd::Zero(1);
  tracker.reset(measured);

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(1);

  const Eigen::VectorXd target = (Eigen::VectorXd(1) << 0.01).finished();
  const Eigen::VectorXd stated = (Eigen::VectorXd(1) << 4.0).finished();
  tracker.update(&target, &stated, measured, 0.0, true, kCycleSeconds, positions, velocities);

  EXPECT_NEAR(positions(0), 0.01, 1e-12);
  EXPECT_NEAR(velocities(0), 0.5, 1e-12);
}

/// A stated velocity is held between arrivals, exactly as a derived one is.
///
/// The two paths differ in where the number comes from, not in when it applies.
/// A cycle with no arrival has nothing new to say about the ceiling, and writing
/// zero there is the defect this whole class exists to avoid.
TEST(ForwardCommand, HoldsAStatedVelocityBetweenArrivals)
{
  ForwardCommandTracker tracker = makeTracker(1.0, 0.2);
  const Eigen::VectorXd measured = Eigen::VectorXd::Zero(1);
  tracker.reset(measured);

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(1);

  const Eigen::VectorXd target = (Eigen::VectorXd(1) << 0.02).finished();
  const Eigen::VectorXd stated = (Eigen::VectorXd(1) << 0.25).finished();
  tracker.update(&target, &stated, measured, 0.0, true, kCycleSeconds, positions, velocities);
  ASSERT_NEAR(velocities(0), 0.25, 1e-12);

  for (int cycle = 0; cycle < 3; ++cycle) {
    velocities.setZero();
    const bool holding = tracker.update(
      &target, &stated, measured, kCycleSeconds, false, kCycleSeconds, positions, velocities);
    EXPECT_FALSE(holding);
    EXPECT_DOUBLE_EQ(velocities(0), 0.25);
  }
}

/// A velocity that belongs to another robot's joints makes the command unusable.
///
/// Holding is the only safe answer: the position is this robot's and the ceiling
/// is not, so following the command would move these joints at a speed that was
/// worked out for different ones. That a command is refused is reported by the
/// holding return, and the caller says so in its log.
TEST(ForwardCommand, RefusesAStatedVelocityOfTheWrongLength)
{
  ForwardCommandTracker tracker = makeTracker(1.0, 0.2);
  const Eigen::VectorXd measured = (Eigen::VectorXd(3) << 0.1, 0.2, 0.3).finished();
  tracker.reset(measured);

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(3);

  const Eigen::VectorXd target = (Eigen::VectorXd(3) << 0.2, 0.2, 0.3).finished();
  const Eigen::VectorXd stated = Eigen::VectorXd::Zero(2);
  const bool holding =
    tracker.update(&target, &stated, measured, 0.0, true, kCycleSeconds, positions, velocities);

  EXPECT_TRUE(holding);
  EXPECT_TRUE(positions.isApprox(measured));
  EXPECT_TRUE(velocities.isZero(0.0));
}

}  // namespace
}  // namespace litearm_vr_teleop
