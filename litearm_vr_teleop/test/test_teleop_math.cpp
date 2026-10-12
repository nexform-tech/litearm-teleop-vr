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
  limits.moving_average_frames = 1;  // no smoothing, so increments are exact
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
  limits.moving_average_frames = 4;
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
  limits.moving_average_frames = 4;
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

}  // namespace
}  // namespace litearm_vr_teleop
