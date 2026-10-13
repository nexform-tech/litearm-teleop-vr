// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.
//
// Tests for the teleoperation arithmetic.
//
// These exist because the failure modes they cover are the ones that are hard to
// see on hardware: a wrong axis sign produces plausible motion in the wrong
// direction, and an increment that does not go to zero when the operator stops
// produces an arm that slowly walks away. Both look like a mechanical problem
// from the outside.

#include <cmath>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "litearm_vr_teleop/teleop_math.hpp"

namespace litearm_vr_teleop
{
namespace
{

constexpr double kTolerance = 1e-9;

/// The identity end effector pose at the origin, used as the "current" pose in
/// tests that only care about the increment itself.
const Eigen::Vector3d kIdentityPosition = Eigen::Vector3d::Zero();
const Eigen::Matrix3d kIdentityRotation = Eigen::Matrix3d::Identity();

/// The tool frame of the arm this package drives, standing at its teleoperation
/// start pose (the `grasp_ready` joint values), expressed in base_link.
///
/// Built from what such a frame is rather than typed in as nine numbers: forward
/// kinematics of that pose gives a tool pointing +y (forward, away from the
/// operator) and 35 degrees down, with the tool's own Y axis horizontal. The
/// numbers are rounded to that description, then built as a rotation, because a
/// matrix of rounded numbers is not quite a rotation and a test comparing
/// against one would be measuring the rounding.
///
/// This is the pose every rotation test below runs at: the wrist roll there is
/// about 95 degrees away from the pose the measured baseline was taken at, which
/// is the whole reason those tests exist.
const Eigen::Matrix3d kStartPoseRotation =
  (Eigen::AngleAxisd(-(M_PI - 55.0 * M_PI / 180.0), Eigen::Vector3d::UnitX()) *
  Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())).toRotationMatrix();

/// The tool frame at the pose the measured baseline was tuned at: the arm's
/// reference home, where the tool's own Y axis points straight down. Forward
/// kinematics again, to the same rounding.
const double kCalibrationTilt = std::atan2(-1.0, -0.029);
const Eigen::Matrix3d kCalibrationPoseRotation =
  Eigen::AngleAxisd(kCalibrationTilt, Eigen::Vector3d::UnitX()).toRotationMatrix();

/// The controller frame -> robot frame correspondence of the measured baseline.
Eigen::Matrix3d controllerToRobot()
{
  return singleArmUprightMapping().translation_remap.transpose();
}

/// The 180 degree flip about X that relates the controller's frame to the
/// tool's at the pose the baseline was measured at: their X axes agree, their Y
/// and Z axes oppose. It is the same three numbers as the reference's measured
/// per axis signs, read as a frame relation.
Eigen::Matrix3d calibratedFlip()
{
  return (Eigen::Matrix3d() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0).finished();
}

TeleopLimits testLimits()
{
  TeleopLimits limits;
  limits.moving_average_samples = 1;  // no smoothing, so increments are exact
  return limits;
}

TEST(TeleopMath, UprightMappingMatchesTheMeasuredBaseline)
{
  const AxisMapping mapping = singleArmUprightMapping();

  // The geometric mode, with no correction: it needs none, because the
  // increment it applies is measured and applied in the robot frame.
  EXPECT_EQ(mapping.rotation_mode, RotationMapping::kWorld);
  EXPECT_TRUE(mapping.rotation_signs.isApprox(Eigen::Vector3d::Ones()));

  // Kept for the tool frame mode, which is the previous behaviour.
  EXPECT_TRUE(mapping.axis_remap.isApprox(Eigen::Matrix3d::Identity()));

  // Row 0: controller +x maps to robot +x. Row 1: robot +y comes from
  // controller -z. Row 2: robot +z comes from controller +y.
  EXPECT_TRUE(
    mapping.translation_remap.isApprox(
      (Eigen::Matrix3d() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0).finished()));
}

/// The property the world frame mode exists for.
///
/// The controller turns about an axis expressed in its own frame; the same axis
/// carries across to the robot frame by the baseline's correspondence. The tool
/// must end up turning about exactly that robot frame axis, by exactly that
/// angle, whatever attitude the tool is in when the frame arrives. That is what
/// "the tool follows the hand" means, and it is what an operator checks first:
/// they turn their hand one way and watch which way the arm goes.
TEST(TeleopMath, WorldModeTurnsTheToolAboutTheWorldAxisTheControllerTurnedAbout)
{
  const Eigen::Matrix3d carry = controllerToRobot();

  // Every axis, because a sign error is one axis at a time.
  const Eigen::Vector3d axes[3] = {
    Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()};
  // Both a pose where the tool's own axes happen to line up with the world and
  // the start pose, where they do not. The second is the one that used to fail.
  const Eigen::Matrix3d poses[2] = {kCalibrationPoseRotation, kStartPoseRotation};

  const double angle = 0.1;
  for (const Eigen::Vector3d & axis : axes) {
    for (const Eigen::Matrix3d & pose : poses) {
      TeleopIncrementMapper mapper(singleArmUprightMapping(), testLimits());
      const Eigen::Vector3d controller(0.0, 1.0, 0.0);
      mapper.start(controller, Eigen::Quaterniond::Identity());

      // The controller turned about one of its own axes.
      const Eigen::Quaterniond turned(Eigen::AngleAxisd(angle, axis));
      const CartesianTarget target =
        mapper.feed(controller, turned, kIdentityPosition, pose);

      // Which is a rotation about the carried axis, by the same angle.
      const Eigen::AngleAxisd expected(Eigen::AngleAxisd(angle, carry * axis));
      const Eigen::Matrix3d increment = target.rotation * pose.transpose();
      EXPECT_TRUE(increment.isApprox(expected.toRotationMatrix(), 1e-9))
        << "axis " << axis.transpose() << " pose\n" << pose;
    }
  }
}

/// The same motion, the same response, whatever pose the arm is in.
///
/// This is the difference between the two modes stated as a test. Under the tool
/// frame mode the response depends on the tool's attitude, because a rotation
/// applied in the tool's own frame is a different world rotation at every pose.
/// At the start pose the wrist is rolled about 95 degrees away from the pose the
/// signs were measured at, and that difference is what the operator was seeing.
TEST(TeleopMath, WorldModeResponseDoesNotDependOnTheArmPose)
{
  const Eigen::Vector3d controller(0.0, 1.0, 0.0);
  const Eigen::Quaterniond turned(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));

  Eigen::Matrix3d world_increment_at_calibration = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d tool_increment_at_calibration = Eigen::Matrix3d::Identity();
  bool first = true;
  for (const Eigen::Matrix3d & pose : {kCalibrationPoseRotation, kStartPoseRotation}) {
    TeleopIncrementMapper world_mapper(singleArmUprightMapping(), testLimits());
    world_mapper.start(controller, Eigen::Quaterniond::Identity());
    const Eigen::Matrix3d world_increment =
      world_mapper.feed(controller, turned, kIdentityPosition, pose).rotation * pose.transpose();

    AxisMapping tool_mapping = singleArmUprightMapping();
    tool_mapping.rotation_mode = RotationMapping::kTool;
    TeleopIncrementMapper tool_mapper(tool_mapping, testLimits());
    tool_mapper.start(controller, Eigen::Quaterniond::Identity());
    const Eigen::Matrix3d tool_increment =
      tool_mapper.feed(controller, turned, kIdentityPosition, pose).rotation * pose.transpose();

    if (first) {
      world_increment_at_calibration = world_increment;
      tool_increment_at_calibration = tool_increment;
      first = false;
      continue;
    }
    // World mode: identical at both poses.
    EXPECT_TRUE(world_increment.isApprox(world_increment_at_calibration, 1e-9));
    // Tool mode: different, which is the defect this mode was replaced for.
    EXPECT_FALSE(tool_increment.isApprox(tool_increment_at_calibration, 1e-3));
  }
}

/// At the attitude the baseline was measured at, the two modes agree.
///
/// This is what makes the change a repair rather than a redesign: the measured
/// numbers were right for the pose they were taken at, and they still produce
/// that motion there. They were only being asked to hold at every other pose,
/// which is the part that could not work.
TEST(TeleopMath, BothModesAgreeAtTheAttitudeTheBaselineWasMeasuredAt)
{
  const Eigen::Matrix3d carry = controllerToRobot();

  // A controller attitude in its own frame, and the tool attitude the baseline
  // relates to it: the controller's frame carried into the robot frame, with the
  // measured flip applied.
  const Eigen::Quaterniond controller_orientation(
    Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.3, -0.5, 0.8).normalized()));
  const Eigen::Matrix3d tool_pose =
    carry * controller_orientation.toRotationMatrix() * calibratedFlip();

  const Eigen::Vector3d controller(0.0, 1.0, 0.0);
  const Eigen::Quaterniond turned(controller_orientation * Eigen::AngleAxisd(
    0.12, Eigen::Vector3d(0.2, 0.9, -0.4).normalized()));

  TeleopIncrementMapper world_mapper(singleArmUprightMapping(), testLimits());
  world_mapper.start(controller, controller_orientation);
  const CartesianTarget world_target =
    world_mapper.feed(controller, turned, kIdentityPosition, tool_pose);

  AxisMapping tool_mapping = singleArmUprightMapping();
  tool_mapping.rotation_mode = RotationMapping::kTool;
  TeleopIncrementMapper tool_mapper(tool_mapping, testLimits());
  tool_mapper.start(controller, controller_orientation);
  const CartesianTarget tool_target =
    tool_mapper.feed(controller, turned, kIdentityPosition, tool_pose);

  EXPECT_TRUE(world_target.rotation.isApprox(tool_target.rotation, 1e-9));
}

TEST(TeleopMath, AnchorFrameProducesNoMotion)
{
  TeleopIncrementMapper mapper(singleArmUprightMapping(), testLimits());
  const Eigen::Vector3d controller(0.1, 1.2, -0.3);
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  mapper.start(controller, orientation);
  const CartesianTarget target = mapper.feed(
    controller, orientation, kIdentityPosition, kIdentityRotation);

  // The anchoring frame contributes nothing: the arm must not move on the press.
  EXPECT_TRUE(target.valid);
  EXPECT_TRUE(target.position.isApprox(kIdentityPosition));
  EXPECT_TRUE(target.rotation.isApprox(kIdentityRotation));
}

TEST(TeleopMath, StillControllerKeepsTheArmStill)
{
  TeleopIncrementMapper mapper(singleArmUprightMapping(), testLimits());
  const Eigen::Vector3d controller(0.0, 1.0, 0.0);
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  mapper.start(controller, orientation);

  // Ten frames at exactly the same pose must produce exactly zero motion in
  // total. This is the property that makes the mapping safe to hand to an
  // operator: the arm cannot drift on its own.
  Eigen::Vector3d reached = kIdentityPosition;
  for (int i = 0; i < 10; ++i) {
    const CartesianTarget target =
      mapper.feed(controller, orientation, reached, kIdentityRotation);
    reached = target.position;
  }
  EXPECT_NEAR((reached - kIdentityPosition).norm(), 0.0, kTolerance);
}

TEST(TeleopMath, ControllerMotionMapsThroughTheTranslationBaseline)
{
  TeleopIncrementMapper mapper(singleArmUprightMapping(), testLimits());
  const Eigen::Vector3d start(0.0, 1.0, 0.0);
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  mapper.start(start, orientation);

  // Controller moved 10 cm to its right (+x).
  const Eigen::Vector3d moved = start + Eigen::Vector3d(0.1, 0.0, 0.0);
  const CartesianTarget target =
    mapper.feed(moved, orientation, kIdentityPosition, kIdentityRotation);

  // Robot forward is +x, so a controller move to the right must show up there
  // and nowhere else.
  EXPECT_NEAR(target.position.x(), 0.1, kTolerance);
  EXPECT_NEAR(target.position.y(), 0.0, kTolerance);
  EXPECT_NEAR(target.position.z(), 0.0, kTolerance);

  // Controller moved 10 cm up (+y): robot up is +z.
  mapper.start(start, orientation);
  const CartesianTarget up =
    mapper.feed(start + Eigen::Vector3d(0.0, 0.1, 0.0), orientation, kIdentityPosition,
      kIdentityRotation);
  EXPECT_NEAR(up.position.y(), 0.0, kTolerance);
  EXPECT_NEAR(up.position.z(), 0.1, kTolerance);

  // Controller pushed forward (-z): the baseline maps that to robot left (+y).
  mapper.start(start, orientation);
  const CartesianTarget forward =
    mapper.feed(start + Eigen::Vector3d(0.0, 0.0, -0.1), orientation, kIdentityPosition,
      kIdentityRotation);
  EXPECT_NEAR(forward.position.y(), 0.1, kTolerance);
  EXPECT_NEAR(forward.position.z(), 0.0, kTolerance);
}

TEST(TeleopMath, PausedMapperCommandsNothingUntilRestarted)
{
  TeleopIncrementMapper mapper(singleArmUprightMapping(), testLimits());
  const Eigen::Vector3d controller(0.0, 1.0, 0.0);
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  mapper.start(controller, orientation);
  mapper.pause();

  const CartesianTarget target =
    mapper.feed(controller + Eigen::Vector3d(0.2, 0.0, 0.0), orientation, kIdentityPosition,
      kIdentityRotation);
  EXPECT_FALSE(target.valid);

  // Restarting re-anchors, so the pose that arrived while paused does not become
  // an increment the moment following resumes.
  mapper.start(controller + Eigen::Vector3d(0.2, 0.0, 0.0), orientation);
  const CartesianTarget restarted = mapper.feed(
    controller + Eigen::Vector3d(0.2, 0.0, 0.0), orientation, kIdentityPosition,
    kIdentityRotation);
  EXPECT_TRUE(restarted.valid);
  EXPECT_NEAR(restarted.position.norm(), 0.0, kTolerance);
}

TEST(TeleopMath, ConstantControllerVelocityTracksAtTheSameVelocity)
{
  TeleopLimits limits;
  limits.moving_average_samples = 4;
  TeleopIncrementMapper mapper(singleArmUprightMapping(), limits);

  const Eigen::Vector3d anchor(0.0, 1.0, 0.0);
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  mapper.start(anchor, orientation);

  // The controller advances 1 cm per frame in its own +x, which the baseline
  // maps straight onto the robot's +x. At steady state the target must advance by
  // the same 1 cm per frame: the averaging smooths the noise inside the motion,
  // it does not attenuate or accumulate the motion itself. Getting this wrong in
  // either direction is felt immediately by an operator, as a lagging or an
  // accelerating arm.
  const double step = 0.01;
  Eigen::Vector3d controller = anchor;
  Eigen::Vector3d reached = kIdentityPosition;
  double previous = reached.x();

  for (int frame = 0; frame < 12; ++frame) {
    controller.x() += step;
    reached = mapper.feed(controller, orientation, reached, kIdentityRotation).position;
    EXPECT_NEAR(reached.x() - previous, step, kTolerance) << "frame " << frame;
    previous = reached.x();
  }
}

TEST(TeleopMath, SmoothingTailAfterASuddenStopIsBounded)
{
  TeleopLimits limits;
  limits.moving_average_samples = 4;
  TeleopIncrementMapper mapper(singleArmUprightMapping(), limits);

  const Eigen::Vector3d anchor(0.0, 1.0, 0.0);
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  mapper.start(anchor, orientation);

  // One 4 cm jump, then the operator stops dead. The window still holds that
  // sample, so the arm keeps advancing for the next few frames. That tail is
  // intended (it is what turns headset noise into smooth motion) but it has to
  // be BOUNDED: a smoothing scheme whose tail does not die out is a following
  // loop that never stops.
  const double jump = 0.04;
  Eigen::Vector3d controller = anchor + Eigen::Vector3d(jump, 0.0, 0.0);
  Eigen::Vector3d reached = kIdentityPosition;
  for (int frame = 0; frame < 40; ++frame) {
    reached = mapper.feed(controller, orientation, reached, kIdentityRotation).position;
  }

  // The theoretical bound is the jump times the harmonic sum of the window
  // length, because the k-th frame after the stop contributes jump / k.
  double harmonic = 0.0;
  for (int k = 1; k <= 4; ++k) {
    harmonic += 1.0 / static_cast<double>(k);
  }
  EXPECT_LE(reached.x(), jump * harmonic + 1e-9);
  EXPECT_GT(reached.x(), jump);  // the jump itself is of course delivered
}

TEST(TeleopMath, DeadbandKeepsAStillHandOutOfTheSolver)
{
  CartesianTarget current;
  current.valid = true;
  current.position = Eigen::Vector3d(0.1, 0.2, 0.3);

  // Tracking noise: a fraction of a millimetre and a fraction of a milliradian.
  CartesianTarget noise = current;
  noise.position += Eigen::Vector3d(0.0002, -0.0001, 0.0);
  noise.rotation =
    current.rotation * Eigen::AngleAxisd(0.0002, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_FALSE(targetDiffers(noise, current, 0.0005, 0.0005));

  // A real request: a millimetre, and a hundredth of a radian.
  CartesianTarget move = current;
  move.position += Eigen::Vector3d(0.001, 0.0, 0.0);
  EXPECT_TRUE(targetDiffers(move, current, 0.0005, 0.0005));

  CartesianTarget turn = current;
  turn.rotation =
    current.rotation * Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitY()).toRotationMatrix();
  EXPECT_TRUE(targetDiffers(turn, current, 0.0005, 0.0005));

  // Zero disables the deadband: any change at all is a change.
  EXPECT_TRUE(targetDiffers(noise, current, 0.0, 0.0));
}

TEST(TeleopMath, LeadIsBoundedWithoutChangingItsDirection)
{
  const Eigen::Vector3d measured_position(0.0, 0.0, 0.4);
  const Eigen::Matrix3d measured_rotation = Eigen::Matrix3d::Identity();

  // Inside the lead: untouched.
  CartesianTarget close;
  close.valid = true;
  close.position = measured_position + Eigen::Vector3d(0.05, 0.0, 0.0);
  close.rotation = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const CartesianTarget kept =
    clampTargetLead(close, measured_position, measured_rotation, 0.15, 0.6);
  EXPECT_TRUE(kept.position.isApprox(close.position));
  EXPECT_TRUE(kept.rotation.isApprox(close.rotation));

  // Beyond it: the position is pulled back onto the sphere of that radius along
  // the direction the operator asked for, and the rotation onto that angle about
  // the same axis.
  CartesianTarget far;
  far.valid = true;
  far.position = measured_position + Eigen::Vector3d(0.6, 0.0, 0.0);
  far.rotation = Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const CartesianTarget bounded =
    clampTargetLead(far, measured_position, measured_rotation, 0.15, 0.6);
  EXPECT_NEAR((bounded.position - measured_position).norm(), 0.15, 1e-9);
  EXPECT_TRUE(
    (bounded.position - measured_position).normalized().isApprox(Eigen::Vector3d::UnitX()));
  const Eigen::AngleAxisd applied(measured_rotation.transpose() * bounded.rotation);
  EXPECT_NEAR(applied.angle(), 0.6, 1e-9);
  EXPECT_NEAR(std::abs(applied.axis().z()), 1.0, 1e-9);
}

TEST(TeleopMath, SignsApplyToTheFrameTheModeReadsIn)
{
  const Eigen::Vector3d controller(0.0, 1.0, 0.0);
  const Eigen::Quaterniond turned(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()));

  // Tool frame mode: the signs act on the tool's own axes, which is what the
  // measured baseline's [1 -1 -1] were. A controller turn about its Y comes out
  // as a tool turn about the tool's -Y.
  AxisMapping tool_mapping = singleArmUprightMapping();
  tool_mapping.rotation_mode = RotationMapping::kTool;
  TeleopIncrementMapper tool_mapper(tool_mapping, testLimits());
  tool_mapper.start(controller, Eigen::Quaterniond::Identity());
  const CartesianTarget tool_target =
    tool_mapper.feed(controller, turned, kIdentityPosition, kIdentityRotation);
  const Eigen::AngleAxisd tool_applied(tool_target.rotation);
  EXPECT_NEAR(tool_applied.angle(), 0.1, 1e-6);
  EXPECT_NEAR(tool_applied.axis().y(), -1.0, 1e-6);

  // World frame mode: the signs act on the robot's axes. The baseline carries
  // the controller's Y to robot +Z, so the Z sign is the one that flips this
  // motion; giving Y the minus instead would leave it alone.
  AxisMapping world_mapping = singleArmUprightMapping();
  world_mapping.rotation_signs = Eigen::Vector3d(1.0, 1.0, -1.0);
  TeleopIncrementMapper world_mapper(world_mapping, testLimits());
  world_mapper.start(controller, Eigen::Quaterniond::Identity());
  const CartesianTarget world_target =
    world_mapper.feed(controller, turned, kIdentityPosition, kIdentityRotation);
  const Eigen::AngleAxisd world_applied(world_target.rotation);
  const Eigen::Vector3d carried = controllerToRobot() * Eigen::Vector3d::UnitY();
  EXPECT_NEAR(carried.z(), 1.0, 1e-9);
  EXPECT_NEAR(world_applied.angle(), 0.1, 1e-6);
  EXPECT_NEAR(world_applied.axis().dot(carried), -1.0, 1e-6);
}

TEST(JointTargetFilter, ClampsOneCycleToTheVelocityLimit)
{
  TeleopLimits limits;
  limits.low_pass_tau = 0.0;  // no smoothing, so only the clamp is under test
  limits.max_joint_velocity = 2.0;
  limits.max_step_rad = 0.08;
  JointTargetFilter filter(limits, 2);

  const Eigen::VectorXd current = Eigen::VectorXd::Zero(2);
  const Eigen::VectorXd target = (Eigen::VectorXd(2) << 1.0, 1.0).finished();

  // 10 ms at 2 rad/s is 0.02 rad, below the absolute ceiling.
  const Eigen::VectorXd applied = filter.apply(target, current, 0.01);
  EXPECT_NEAR(applied(0), 0.02, kTolerance);

  // 100 ms at 2 rad/s would be 0.2 rad, but the absolute ceiling caps the step at
  // 0.08 rad: a long cycle must not authorise an arbitrarily large move.
  JointTargetFilter long_cycle(limits, 2);
  const Eigen::VectorXd applied_long = long_cycle.apply(target, current, 0.1);
  EXPECT_NEAR(applied_long(1), 0.08, kTolerance);
}

TEST(JointTargetFilter, SeededFromTheFirstTargetNotFromTheLowPassState)
{
  TeleopLimits limits;
  limits.low_pass_tau = 0.02;
  JointTargetFilter filter(limits, 1);

  const Eigen::VectorXd current = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd target = (Eigen::VectorXd(1) << 0.5).finished();

  // First call seeds the filter at the target; the output is then the step from
  // the current position, clamped. It must not be zero (which would look like a
  // filter that never starts) and must not be the full target.
  const Eigen::VectorXd applied = filter.apply(target, current, 0.02);
  EXPECT_GT(applied(0), 0.0);
  EXPECT_LT(applied(0), 0.5);
}

TEST(JointTargetFilter, ConstantStepIsTrackedWithThatSameStep)
{
  TeleopLimits limits;
  limits.low_pass_tau = 0.02;
  limits.max_joint_velocity = 2.0;
  limits.max_step_rad = 0.08;
  JointTargetFilter filter(limits, 1);

  // A target that stays a constant 1 mrad ahead of the arm, which is what a hand
  // moving at a constant speed produces. Once the filter settles, the arm must
  // advance by exactly that 1 mrad per cycle. Anything less is a gain error, and
  // an operator feels a gain error as the robot refusing to follow their hand.
  const double step = 1e-3;
  Eigen::VectorXd current = Eigen::VectorXd::Zero(1);
  double applied = 0.0;
  for (int cycle = 0; cycle < 400; ++cycle) {
    const Eigen::VectorXd target = current + Eigen::VectorXd::Constant(1, step);
    const Eigen::VectorXd next = filter.apply(target, current, 0.01);
    applied = next(0) - current(0);
    current = next;
  }
  EXPECT_NEAR(applied, step, 1e-9);
}

TEST(JointTargetFilter, RefusesToMoveWhenSizesDisagree)
{
  JointTargetFilter filter(testLimits(), 3);
  const Eigen::VectorXd current = Eigen::VectorXd::Zero(3);
  const Eigen::VectorXd wrong_size = Eigen::VectorXd::Zero(2);

  EXPECT_TRUE(filter.apply(wrong_size, current, 0.01).isApprox(current));
}

TEST(JointRateLimiter, KeepsTheCommandStreamInsideItsLimits)
{
  JointRateLimits limits;
  limits.max_velocity = 2.0;
  limits.max_acceleration = 20.0;
  limits.max_jerk = 1000.0;
  JointRateLimiter limiter(limits, 1);

  // A target that jumps is the worst case a limiter can be handed, and the one
  // the firmware cares about: it reads the velocity field as the slew ceiling of
  // the position reference, so a command stream whose speed steps asks it for a
  // step in speed and pins that field at its own limit. What matters is that the
  // stream that comes out is differentiable twice over.
  const double dt = 0.01;
  const Eigen::VectorXd target = (Eigen::VectorXd(1) << 0.3).finished();
  Eigen::VectorXd current = Eigen::VectorXd::Zero(1);

  double previous_velocity = 0.0;
  double previous_acceleration = 0.0;
  for (int cycle = 0; cycle < 200; ++cycle) {
    const Eigen::VectorXd next = limiter.apply(target, current, dt);
    const double velocity = (next(0) - current(0)) / dt;
    const double acceleration = (velocity - previous_velocity) / dt;
    EXPECT_LE(std::abs(velocity), limits.max_velocity + 1e-9);
    EXPECT_LE(std::abs(acceleration), limits.max_acceleration + 1e-9);
    // The jerk bound is what makes the speed curve an S rather than a ramp, and
    // it is the property the whole class exists for.
    EXPECT_LE(std::abs(acceleration - previous_acceleration), limits.max_jerk * dt + 1e-9);
    EXPECT_GE(next(0), current(0));            // monotone: no overshoot, no oscillation
    EXPECT_LE(next(0), target(0) + 1e-12);     // and never past the target
    previous_velocity = velocity;
    previous_acceleration = acceleration;
    current = next;
  }

  EXPECT_NEAR(current(0), target(0), 1e-6);
}

TEST(JointRateLimiter, TracksAMovingTargetWithoutLaggingBehindIt)
{
  JointRateLimits limits;
  JointRateLimiter limiter(limits, 1);

  // A target moving at a constant rate, which is what a hand moving steadily
  // produces. The limiter must not turn that into a permanent offset: smoothing
  // that costs gain is what makes an operator say the robot moves less than
  // their hand, and the velocity form mapping's whole point is that the arm
  // ends up where the hand is.
  const double dt = 0.01;
  const double rate = 0.5;   // rad/s, a quarter of the rated speed
  Eigen::VectorXd current = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd target = Eigen::VectorXd::Zero(1);
  for (int cycle = 0; cycle < 400; ++cycle) {
    target(0) += rate * dt;
    current = limiter.apply(target, current, dt);
  }

  EXPECT_NEAR(current(0), target(0), 1e-3);
}

TEST(JointRateLimiter, KeepsAStillHandStill)
{
  JointRateLimits limits;
  JointRateLimiter limiter(limits, 1);

  const double dt = 0.01;
  Eigen::VectorXd current = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd target = (Eigen::VectorXd(1) << 0.05).finished();
  for (int cycle = 0; cycle < 300; ++cycle) {
    current = limiter.apply(target, current, dt);
  }
  EXPECT_NEAR(current(0), target(0), 1e-9);

  // Holding a reached target produces exactly no motion: a still hand does not
  // produce a limit cycle, which is the other half of what this class is for.
  double total = 0.0;
  for (int cycle = 0; cycle < 100; ++cycle) {
    const Eigen::VectorXd next = limiter.apply(target, current, dt);
    total += std::abs(next(0) - current(0));
    current = next;
  }
  EXPECT_NEAR(total, 0.0, 1e-12);
}

TEST(JointRateLimiter, RefusesATargetForADifferentNumberOfJoints)
{
  JointRateLimits limits;
  JointRateLimiter limiter(limits, 3);
  const Eigen::VectorXd current = Eigen::VectorXd::Zero(3);
  const Eigen::VectorXd wrong_size = Eigen::VectorXd::Zero(2);
  EXPECT_TRUE(limiter.apply(wrong_size, current, 0.01).isApprox(current));
}

/// The limiter works its speed limit out in the frame of "towards the target",
/// because the limit is a distance and a distance has no sign. The acceleration
/// has to be turned back into the joint's frame before it is applied. Getting
/// that wrong is a limiter that only ever moves one way: a joint whose target is
/// below it is told to accelerate upwards, the step guard refuses the step, and
/// the arm sits still. This is the test that catches it.
TEST(JointRateLimiter, MovesTowardsATargetBelowItJustAsWellAsAboveIt)
{
  JointRateLimits limits;
  limits.max_velocity = 2.0;
  limits.max_acceleration = 20.0;
  limits.max_jerk = 1000.0;
  JointRateLimiter limiter(limits, 1);

  const double dt = 0.01;
  const Eigen::VectorXd target = (Eigen::VectorXd(1) << -0.3).finished();
  Eigen::VectorXd current = Eigen::VectorXd::Zero(1);

  double previous_velocity = 0.0;
  double previous_acceleration = 0.0;
  for (int cycle = 0; cycle < 200; ++cycle) {
    const Eigen::VectorXd next = limiter.apply(target, current, dt);
    const double velocity = (next(0) - current(0)) / dt;
    const double acceleration = (velocity - previous_velocity) / dt;
    EXPECT_LE(std::abs(velocity), limits.max_velocity + 1e-9);
    EXPECT_LE(std::abs(acceleration), limits.max_acceleration + 1e-9);
    EXPECT_LE(std::abs(acceleration - previous_acceleration), limits.max_jerk * dt + 1e-9);
    // Monotone towards the target, in the negative direction.
    EXPECT_LE(next(0), current(0));
    EXPECT_GE(next(0), target(0) - 1e-12);
    previous_velocity = velocity;
    previous_acceleration = acceleration;
    current = next;
  }
  EXPECT_NEAR(current(0), target(0), 1e-6);
}

/// A reversal: the operator moves one way, then the other, and the target ends
/// up behind the arm. The command has to come back through zero and go the other
/// way, smoothly, instead of stalling where it is.
TEST(JointRateLimiter, ReversesThroughZeroWithoutStalling)
{
  JointRateLimits limits;
  JointRateLimiter limiter(limits, 1);

  const double dt = 0.01;
  Eigen::VectorXd current = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd target = Eigen::VectorXd::Zero(1);
  for (int cycle = 0; cycle < 50; ++cycle) {          // up
    target(0) = 0.2;
    current = limiter.apply(target, current, dt);
  }
  EXPECT_NEAR(current(0), 0.2, 1e-6);

  double previous = current(0);
  for (int cycle = 0; cycle < 200; ++cycle) {         // and back down, past zero
    target(0) = -0.2;
    current = limiter.apply(target, current, dt);
    // Never moves away from the target, in either direction.
    const double towards = (target(0) - previous) >= 0.0 ? 1.0 : -1.0;
    EXPECT_GE((current(0) - previous) * towards, -1e-12);
    previous = current(0);
  }
  EXPECT_NEAR(current(0), -0.2, 1e-6);
}

/// The tick generator runs at the rate it was given, whatever the caller does.
///
/// This is the property that keeps the target on the node's clock: the caller's
/// lateness changes when in the period a tick is noticed, never how many ticks
/// there are per second. Ten seconds is deliberate - the schedule is an absolute
/// grid computed in floating point, and a grid that loses a fraction of a period
/// per tick still looks right over a tenth of a second.
TEST(ControlTick, ProducesOneTickPerPeriod)
{
  ControlTick tick(100.0);
  int ticks = 0;
  double first = 0.0;
  double last = 0.0;
  for (int ms = 0; ms <= 10000; ++ms) {               // 1 ms poll, 100 Hz tick
    const double now = static_cast<double>(ms) / 1000.0;
    if (tick.due(now)) {
      if (ticks == 0) {
        first = now;
      }
      last = now;
      ++ticks;
    }
  }
  // The first call starts the schedule rather than firing, so ten seconds of
  // calls hold one thousand ticks, the first of them one period in and the last
  // of them on the second.
  EXPECT_EQ(ticks, 1000);
  EXPECT_NEAR(first, 0.01, 1e-9);
  EXPECT_NEAR(last, 10.0, 1e-9);
}

/// A caller that is late is not a caller that gets extra ticks.
///
/// A thread can be descheduled for longer than a period - a garbage collection on
/// the desktop, another process taking the core - and the ticks it missed are
/// ticks for a target that has already moved on. Replaying them would ask for the
/// same target several times in a row.
TEST(ControlTick, DoesNotCatchUpAfterAMissedPeriod)
{
  ControlTick tick(100.0);
  EXPECT_FALSE(tick.due(0.0));

  // 240 ms later: the twenty four periods in between are gone, not delayed.
  int ticks = 0;
  for (double now = 0.25; now <= 0.252; now += 0.0005) {
    ticks += tick.due(now) ? 1 : 0;
  }
  EXPECT_EQ(ticks, 1);
  // And the schedule carries on from where it was, one period after that tick
  // rather than one period after the late call.
  EXPECT_TRUE(tick.due(0.27));
}

/// A rate that cannot be honoured is repaired.
///
/// The same reading every other limit in this file takes: a zero would divide by
/// zero, and refusing to tick at all would look from the outside exactly like a
/// node that started and then quietly stopped moving the arm.
TEST(ControlTick, RepairsAnImpossibleRate)
{
  ControlTick zero(0.0);
  EXPECT_FALSE(zero.due(0.0));
  EXPECT_TRUE(zero.due(1.0));
}

/// A rate is a rate: the same hand speed gives the same twist whatever the tick.
///
/// This is what the velocity following mode is built on, and the property the
/// increment form cannot have. The increment is the hand's motion over one
/// arrival, so a late arrival carries more of it and a step sized by it is a step
/// sized by the link's timing; the rate divides by the time the motion took, so
/// the number that reaches the Jacobian is the operator's speed and nothing else.
TEST(TeleopMath, HandRateIsIndependentOfTheTickLength)
{
  TeleopLimits limits = testLimits();
  TeleopIncrementMapper mapper(singleArmUprightMapping(), limits);

  const Eigen::Vector3d start = (Eigen::Vector3d() << 0.4, 0.2, 0.3).finished();
  const Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
  // A hand travelling at 0.2 m/s along the robot's x, expressed in the headset
  // frame: the mapping carries the robot axis back into the controller's.
  const Eigen::Matrix3d hand_from_robot = singleArmUprightMapping().translation_remap;
  const Eigen::Vector3d speed_in_hand_frame = hand_from_robot * Eigen::Vector3d(0.2, 0.0, 0.0);

  mapper.start(start, attitude);

  TeleopIncrementMapper quick(singleArmUprightMapping(), limits);
  quick.start(start, attitude);
  const CartesianTwist fast_tick = quick.feedRate(start + speed_in_hand_frame * 0.01, attitude, 0.01);

  TeleopIncrementMapper slow(singleArmUprightMapping(), limits);
  slow.start(start, attitude);
  const CartesianTwist late_tick = slow.feedRate(start + speed_in_hand_frame * 0.025, attitude, 0.025);

  ASSERT_TRUE(fast_tick.valid);
  ASSERT_TRUE(late_tick.valid);
  EXPECT_NEAR(fast_tick.linear.x(), 0.2, 1e-9);
  EXPECT_NEAR(late_tick.linear.x(), 0.2, 1e-9);
  EXPECT_NEAR(late_tick.linear.y(), 0.0, 1e-12);
  EXPECT_NEAR(late_tick.linear.z(), 0.0, 1e-12);
}

/// The window averages rates over the time it covered, not over its samples.
///
/// With a uniform tick the two are the same thing, which is why this is easy to
/// get wrong and never notice; with one late tick they are not, and the average
/// of the samples would understate the speed of a hand that accelerated during
/// the long one - the same class of error as dividing a step by the wrong dt.
TEST(TeleopMath, HandRateAveragesOverTheTimeItCovered)
{
  TeleopLimits limits = testLimits();
  limits.moving_average_samples = 2;
  TeleopIncrementMapper mapper(singleArmUprightMapping(), limits);
  const Eigen::Matrix3d hand_from_robot = singleArmUprightMapping().translation_remap;

  const Eigen::Vector3d start = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
  mapper.start(start, attitude);

  // First sample: 0.1 m/s for 10 ms. Second: 0.3 m/s for 30 ms. Over the two the
  // hand covered 0.001 + 0.009 = 0.01 m in 40 ms, so it travelled at 0.25 m/s.
  mapper.feedRate(start + hand_from_robot * Eigen::Vector3d(0.001, 0, 0), attitude, 0.01);
  const CartesianTwist twist = mapper.feedRate(
    start + hand_from_robot * Eigen::Vector3d(0.010, 0, 0), attitude, 0.03);

  ASSERT_TRUE(twist.valid);
  EXPECT_NEAR(twist.linear.x(), 0.25, 1e-9);
}

/// The profile starts from rest and never steps.
///
/// The first moment of following is the one the operator is most likely to
/// notice, and a command that jumped to its full speed there would be a step in
/// the field the firmware reads as the slew ceiling - the defect this mode exists
/// to remove, moved to the start of the session.
TEST(TeleopMath, VelocityProfileRampsFromRest)
{
  JointVelocityLimits limits;
  limits.max_velocity = 2.0;
  limits.max_acceleration = 20.0;
  limits.filter_tau = 0.03;
  JointVelocityProfile profile(limits, 1);

  const Eigen::VectorXd target = (Eigen::VectorXd(1) << 1.0).finished();
  const double dt = 0.01;
  Eigen::VectorXd velocity = profile.apply(target, dt);
  EXPECT_GT(velocity(0), 0.0);
  double previous = velocity(0);
  for (int cycle = 0; cycle < 60; ++cycle) {
    velocity = profile.apply(target, dt);
    // Bounded by the acceleration ceiling on the way up...
    EXPECT_LE(velocity(0) - previous, limits.max_acceleration * dt + 1e-12);
    previous = velocity(0);
    EXPECT_LE(velocity(0), limits.max_velocity + 1e-12);
  }
  // ...and it gets there: the limit is a ramp, not a brake.
  EXPECT_NEAR(velocity(0), 1.0, 1e-6);
}

/// A speed above the ceiling is cut, in both directions.
TEST(TeleopMath, VelocityProfileClampsToTheCeiling)
{
  JointVelocityLimits limits;
  limits.max_velocity = 0.5;
  limits.max_acceleration = 1000.0;
  limits.filter_tau = 0.0;
  JointVelocityProfile profile(limits, 2);

  const Eigen::VectorXd target = (Eigen::VectorXd(2) << 3.0, -3.0).finished();
  Eigen::VectorXd velocity = profile.apply(target, 0.01);
  for (int cycle = 0; cycle < 100; ++cycle) {
    velocity = profile.apply(target, 0.01);
  }
  EXPECT_NEAR(velocity(0), 0.5, 1e-9);
  EXPECT_NEAR(velocity(1), -0.5, 1e-9);
}

/// Integrating is the whole of "where the reference goes".
TEST(TeleopMath, IntegratesAVelocityIntoAPosition)
{
  const Eigen::VectorXd position = (Eigen::VectorXd(1) << 0.1).finished();
  const Eigen::VectorXd velocity = (Eigen::VectorXd(1) << 0.2).finished();
  const Eigen::VectorXd next = integrateJointVelocity(
    position, velocity, 0.05, Eigen::VectorXd(), Eigen::VectorXd());
  EXPECT_NEAR(next(0), 0.11, 1e-12);
}

/// A joint at its limit stops there rather than running past it.
///
/// The hardware refuses the motion anyway, so a command that kept climbing
/// outside the range would only build a difference that has to be given back the
/// moment the operator reverses - which is the accumulation the velocity form
/// exists to avoid.
TEST(TeleopMath, IntegrationStopsAtTheJointLimits)
{
  const Eigen::VectorXd position = (Eigen::VectorXd(2) << 0.0, 0.0).finished();
  const Eigen::VectorXd velocity = (Eigen::VectorXd(2) << 10.0, -10.0).finished();
  const Eigen::VectorXd lower = (Eigen::VectorXd(2) << -0.5, -0.5).finished();
  const Eigen::VectorXd upper = (Eigen::VectorXd(2) << 0.5, 0.5).finished();

  const Eigen::VectorXd next = integrateJointVelocity(position, velocity, 0.05, lower, upper);
  EXPECT_NEAR(next(0), 0.5, 1e-12);
  EXPECT_NEAR(next(1), -0.5, 1e-12);
}

/// The tool tip moves faster than the wrist, by exactly the lever arm.
///
/// The operator's hand drives the tool tip and the arm's Jacobian is evaluated at
/// the end effector link, so the twist has to be carried between the two; without
/// this the arm turns at the right speed and translates at the wrong one,
/// wherever the tool is offset from the link.
TEST(TeleopMath, CarriesATwistAcrossAnOffset)
{
  CartesianTwist twist;
  twist.linear = Eigen::Vector3d(0.1, 0.2, 0.3);
  twist.angular = Eigen::Vector3d(0.0, 0.0, 1.0);   // 1 rad/s about z
  twist.valid = true;

  const Eigen::Vector3d offset(0.2, 0.0, 0.0);      // 20 cm along x
  const CartesianTwist at_origin = twistAtOffset(twist, offset);

  // A point 20 cm along x on a body spinning about z moves at +0.2 m/s in y, so
  // the motion seen at the origin has 0.2 m/s less of it.
  EXPECT_NEAR(at_origin.linear.x(), 0.1, 1e-12);
  EXPECT_NEAR(at_origin.linear.y(), 0.2 - 0.2, 1e-12);
  EXPECT_NEAR(at_origin.linear.z(), 0.3, 1e-12);
  EXPECT_NEAR(at_origin.angular.z(), 1.0, 1e-12);
  EXPECT_TRUE(at_origin.valid);
}

/// The lead guard leaves a small error alone and stops a large one.
///
/// The velocity form integrates, so without this a lagging arm accumulates the
/// operator's motion in the reference until a joint limit stops it - measured on
/// the fake firmware as a command asking for the full 2 rad/s on a quarter of the
/// samples of an ordinary recorded episode.
TEST(TeleopMath, LeadScaleGuardsTheIntegratedReference)
{
  // Inside the soft band: the arm's ordinary tracking error, not a runaway.
  EXPECT_DOUBLE_EQ(leadScale((Eigen::VectorXd(2) << 0.01, -0.02).finished(), 0.2), 1.0);
  // At the margin: the command is zero, so the reference stops running away.
  EXPECT_DOUBLE_EQ(leadScale((Eigen::VectorXd(1) << 0.2).finished(), 0.2), 0.0);
  EXPECT_DOUBLE_EQ(leadScale((Eigen::VectorXd(1) << -0.3).finished(), 0.2), 0.0);
  // In between: a continuous ramp, so the scaling cannot chatter.
  const double half = leadScale((Eigen::VectorXd(1) << 0.15).finished(), 0.2);
  EXPECT_GT(half, 0.0);
  EXPECT_LT(half, 1.0);
  // The worst joint decides for all of them, so the command keeps its direction.
  const double mixed = leadScale((Eigen::VectorXd(2) << 0.001, 0.15).finished(), 0.2);
  EXPECT_NEAR(mixed, half, 1e-12);
  // Disabled, and an empty command.
  EXPECT_DOUBLE_EQ(leadScale((Eigen::VectorXd(1) << 9.0).finished(), 0.0), 1.0);
  EXPECT_DOUBLE_EQ(leadScale(Eigen::VectorXd(), 0.2), 1.0);
}

/// A trigger resting near its threshold must not oscillate.
///
/// Measured on the rig this was written for: an analogue trigger nobody was
/// touching produced an open/close pair every half second, at exactly the minimum
/// close window, because the value wandered across a single threshold. A debounce
/// cannot fix that - it only asks the signal to stay put for 50 ms, which a slow
/// wobble does - so the press and the release happen at two different values.
TEST(TeleopMath, TriggerHasHysteresis)
{
  const double press_at = 0.5;
  const double hysteresis = 0.1;

  // From released, it takes a value above the threshold to press.
  EXPECT_FALSE(triggerPressed(0.50, false, press_at, hysteresis));
  EXPECT_TRUE(triggerPressed(0.51, false, press_at, hysteresis));

  // Once pressed, a wobble below the threshold keeps it pressed...
  EXPECT_TRUE(triggerPressed(0.45, true, press_at, hysteresis));
  EXPECT_TRUE(triggerPressed(0.41, true, press_at, hysteresis));

  // ...and only a release past the far threshold opens it.
  EXPECT_FALSE(triggerPressed(0.39, true, press_at, hysteresis));

  // The band is directional: released inside it stays released, so a value that
  // sits in the band cannot produce an edge at all.
  EXPECT_FALSE(triggerPressed(0.45, false, press_at, hysteresis));

  // Zero hysteresis is the bare threshold, which is what oscillated.
  EXPECT_FALSE(triggerPressed(0.499, false, press_at, 0.0));
  EXPECT_TRUE(triggerPressed(0.501, false, press_at, 0.0));
  EXPECT_FALSE(triggerPressed(0.499, true, press_at, 0.0));
}

}  // namespace
}  // namespace litearm_vr_teleop
