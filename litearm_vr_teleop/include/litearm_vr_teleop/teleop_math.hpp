// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.
//
// Pure teleoperation arithmetic. No ROS headers, no robot model, no time source
// other than the dt the caller passes in, so every function here is directly
// unit testable.
//
// The numbers come from vr_control's metavr/src/litearm_teleop/teleop_sim.py,
// which is where they were measured against a real headset and a real arm. They
// are constants, not derived quantities: the only honest way to change them is
// to re-measure, so they are named after what they describe rather than
// inlined.

#ifndef LITEARM_VR_TELEOP__TELEOP_MATH_HPP_
#define LITEARM_VR_TELEOP__TELEOP_MATH_HPP_

#include <cstddef>
#include <deque>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace litearm_vr_teleop
{

/// Which frame the controller's rotation increment is interpreted in.
///
/// The two differ in what they hold fixed, and that decides what an operator
/// sees when they turn their hand:
///
///   kWorld  the increment is the controller's rotation in the world, applied
///           to the tool in the world. Turn the controller about a world axis
///           and the tool turns about that same world axis, by the same angle,
///           whatever pose the arm is in and however the controller is held.
///   kTool   the increment is the controller's rotation in its own frame,
///           applied to the tool in the tool's own frame, so the tool copies
///           the hand axis for axis. That is only the same motion as kWorld
///           while the tool's frame and the controller's frame stand in one
///           specific relation to each other.
///
/// kWorld is the default because that relation is not a property of the
/// installation, it is a property of the arm's pose. The measured baseline
/// below was tuned with the wrist rolled so that the tool's own Y axis pointed
/// straight down; at any other wrist roll the tool frame is rolled with respect
/// to the world by the difference, and kTool hands the operator a response
/// rotated by exactly that much. Under kWorld the same measured motion is
/// reproduced exactly and a different wrist roll cannot change it.
enum class RotationMapping
{
  kWorld,
  kTool,
};

/// Correspondence between the WebXR controller frame and the robot frame.
///
/// The translation matrix is applied as a similarity transform of the relative
/// motion: `pinv(H) * delta * H`, where H carries only the rotation part.
/// Writing it this way means an axis permutation (a swap of two columns plus a
/// sign) is expressed as one matrix instead of hand applied component by
/// component, which is where sign errors normally come from.
struct AxisMapping
{
  /// Frame the controller's rotation increment is read in. See RotationMapping.
  RotationMapping rotation_mode = RotationMapping::kWorld;
  /// Rotation correspondence between the controller's own frame and the tool's
  /// own frame. Used by kTool only: kWorld needs no correspondence at all,
  /// because the increment it applies is already expressed in the robot frame.
  Eigen::Matrix3d axis_remap = Eigen::Matrix3d::Identity();
  /// Frame correspondence between the WebXR controller frame and the robot base
  /// frame, read as "column = robot axis, row = controller axis": a controller
  /// frame vector becomes a robot frame vector after multiplying by
  /// `translation_remap.transpose()`.
  ///
  /// It is one correspondence for both translation and rotation - the headset's
  /// reference frame is gravity aligned but yawed with respect to the robot -
  /// so kWorld also uses it to carry the controller's attitude into the robot
  /// frame. It is per installation: it depends on which way the operator stands
  /// relative to the arm, which is why it cannot be shared with a rig that is
  /// laid out differently.
  Eigen::Matrix3d translation_remap = Eigen::Matrix3d::Identity();
  /// Per axis sign applied to the rotation vector the mode produces, component
  /// 0/1/2 being the frame that mode reads in: robot base axes for kWorld, tool
  /// axes for kTool.
  ///
  /// A correction on top of the mode's own baseline, not a replacement for it:
  /// kWorld's baseline is nothing (the geometry is already right), kTool's is
  /// the measured `tool_rotation_signs` below. All ones therefore means "leave
  /// this mode as measured", which is what makes switching modes at run time
  /// give the other measured behaviour rather than a mode with a sign silently
  /// missing.
  Eigen::Vector3d rotation_signs = Eigen::Vector3d::Ones();
  /// The measured per axis signs of the tool frame mode, used by kTool only and
  /// composed with the correction above. These are the reference's numbers:
  /// they relate the controller's frame to the tool's at the one pose they were
  /// measured at, which is why kTool is not the default. See RotationMapping.
  Eigen::Vector3d tool_rotation_signs = Eigen::Vector3d(1.0, -1.0, -1.0);
};

/// Mapping for one upright arm driven by the right controller.
///
/// The upright arm's end effector points along a different world axis than the
/// side mounted variant, so the frame correspondence is per installation and
/// cannot be shared. This one is the measured upright baseline.
AxisMapping singleArmUprightMapping();

/// Tuning of the incremental mapping and of the joint filter.
struct TeleopLimits
{
  /// Absolute ceiling on the joint step of one cycle, independent of dt. This
  /// is not the normal rate limit, it is the floor under an inverse kinematics
  /// jump: whatever else goes wrong, one cycle may not move a joint further
  /// than this.
  double max_step_rad = 0.08;
  /// Normal rate limit in rad/s, matched to the follow server in vr_control so
  /// that the commanded motion has the same shape there and here.
  double max_joint_velocity = 2.0;
  /// Time constant of the joint target low pass in seconds, roughly an 8 Hz
  /// corner. Headset tracking noise is high frequency and small; inverse
  /// kinematics amplifies it, and what reaches the motors is joint jitter. The
  /// low pass keeps the slow component, which is the operator's intent.
  double low_pass_tau = 0.02;
  /// Frames in the moving average of the Cartesian increment. Averaging before
  /// the inverse kinematics rather than after matters: it makes the target pose
  /// smooth, so the solver sees a consistent seed sequence instead of chasing
  /// noise.
  std::size_t moving_average_frames = 10;
};

/// Cartesian end effector target produced by one teleoperation frame.
struct CartesianTarget
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  /// False until the first frame after `start()`. The anchoring frame produces
  /// no increment, so there is nothing to command yet.
  bool valid = false;
};

/// Whether a freshly mapped target has moved far enough from the one already
/// being followed to be worth acting on.
///
/// The deadband exists because a hand that is not moving is not a still input:
/// headset tracking noise reaches the mapper as sub-millimetre, sub-milliradian
/// pose changes every frame, and each one becomes a slightly different Cartesian
/// target. Left alone, the inverse kinematics re-solves for every one of them
/// and the joint answer moves by whatever that frame's noise was, which the
/// controller then reports as a velocity. Below the deadband the target is
/// simply kept, so a still hand produces no new solve at all.
///
/// The cost is a threshold, and it has to be understood in the mode it is used
/// in: the target is re-based on the measured pose every frame, so an increment
/// below the deadband is not accumulated, it is dropped. A deadband expressed in
/// metres therefore acts as a velocity threshold of
/// `deadband / headset frame period` (0.5 mm at 72 Hz is 3.6 cm/s). That is why
/// the default is zero: with the reference on the commanded pose the increments
/// do accumulate and a deadband is safe, with the reference on the measured pose
/// a large one would be the very "nothing happens until I move my hand a long
/// way" the deadband is meant to avoid.
bool targetDiffers(
  const CartesianTarget & candidate, const CartesianTarget & current,
  double position_deadband, double rotation_deadband);

/// Pull a target back toward a measured pose when it has been allowed to run
/// ahead of it by more than `max_lead`.
///
/// The windup guard for the mode that accumulates on the commanded pose. Without
/// it, a target the arm cannot reach - a blocked link, a refused solution, an
/// operator who keeps pushing while the arm holds - would keep collecting the
/// hand's motion and then hand the arm a large jump once the obstruction clears.
/// Positions beyond the lead are projected back onto the sphere of that radius,
/// and rotations beyond the rotation lead onto the corresponding angle, which
/// keeps the direction the operator asked for and only bounds how far ahead the
/// arm is allowed to be asked to go. A lead of zero or less disables the clamp.
CartesianTarget clampTargetLead(
  const CartesianTarget & target, const Eigen::Vector3d & measured_position,
  const Eigen::Matrix3d & measured_rotation, double max_lead_position,
  double max_lead_rotation);

/// Velocity-form (incremental) mapping from headset poses to an end effector
/// target.
///
/// The headset never commands an absolute robot pose. Pressing the grip anchors
/// the mapping on the headset pose of that instant, and every later frame
/// contributes `pinv(previous_controller) * current_controller`. A still hand
/// therefore contributes zero and the arm stops; there is no drift term that
/// would let the arm keep travelling after the operator stops. Releasing the
/// grip pauses without losing the arm's pose, and feeding zero increments
/// through the average while paused means following resumes from a small step
/// rather than a jump.
class TeleopIncrementMapper
{
public:
  TeleopIncrementMapper(AxisMapping mapping, TeleopLimits limits);

  /// Anchor on this headset pose and drop the increment history.
  void start(const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation);

  /// Pause. The anchor is kept so a later `start()` is not required, but no
  /// further increments are produced until `start()` is called again.
  void pause();

  /// Drop the anchor and the history. Used when the link to the headset drops:
  /// the next grip press has to re-anchor, because the pose after a reconnect
  /// is not continuous with the pose before it.
  void reset();

  /// Replace the mapping without touching the anchor.
  ///
  /// What this is for: the frame correspondence and the per axis signs are
  /// calibration, and the person who can tell whether they are right is the one
  /// holding the controller. Changing them while following has to be possible
  /// without a restart, so the mapping is settable at run time.
  ///
  /// The increment history is dropped, because its entries were measured in the
  /// frame the previous mapping read; averaging across the two would apply a
  /// mixture that is neither mapping. The anchor and the active flag stay, so
  /// the next frame produces one increment under the new mapping and the arm
  /// does not jump.
  void setMapping(const AxisMapping & mapping);

  bool active() const { return active_; }

  /// Feed one headset frame and produce the end effector target.
  ///
  /// `ee_position` and `ee_rotation` are the CURRENT end effector pose, taken
  /// from forward kinematics of the measured joints rather than from the
  /// previous target. That is what keeps the increment relative to where the
  /// arm actually is, so a target that was clipped or refused does not silently
  /// accumulate into the next frame's command.
  CartesianTarget feed(
    const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation,
    const Eigen::Vector3d & ee_position, const Eigen::Matrix3d & ee_rotation);

private:
  /// Turn a headset pose into a homogeneous transform.
  static Eigen::Matrix4d poseToHomogeneous(
    const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation);

  AxisMapping mapping_;
  TeleopLimits limits_;
  bool active_ = false;
  Eigen::Matrix4d previous_controller_ = Eigen::Matrix4d::Identity();
  std::deque<Eigen::Matrix<double, 6, 1>> history_;
};

/// Joint space filter applied to the inverse kinematics result: low pass on the
/// rate, then a per cycle clamp.
///
/// The two stages do different jobs and are kept together because they act on the
/// same signal in a fixed order. The low pass removes the cycle to cycle jitter
/// that headset tracking noise turns into after the solver amplifies it; the
/// clamp bounds what a solver mistake may cost in a single cycle. Clamping after
/// the filter is the meaningful order: clamping first would let the filter ramp
/// toward a step that was already refused.
///
/// The filter runs on the requested step rather than on the absolute target, and
/// that is a deliberate departure from the reference implementation, which
/// smooths the target. Smoothing the target costs gain: the next cycle's target
/// is one increment ahead of wherever the arm actually got to, so the arm
/// advances by a fraction of the requested motion, and the fraction depends on
/// the time constant and the cycle rate. Measured on the mock hardware, a 12 cm
/// controller move was delivered as 8 cm. An operator notices that immediately as
/// "the robot moves less than my hand", and no amount of re-tuning makes it a
/// number they could learn. Filtering the step leaves the gain at one.
class JointTargetFilter
{
public:
  JointTargetFilter(TeleopLimits limits, std::size_t dof);

  /// Returns the joint values to command for this cycle.
  ///
  /// `current` is the measured joint state, not the previous command: the
  /// question being answered is how far this cycle may move from where the arm
  /// is.
  Eigen::VectorXd apply(
    const Eigen::VectorXd & target, const Eigen::VectorXd & current, double dt);

  /// Forget the low pass state. The next call seeds from its target, which is
  /// what a fresh start after a pause wants.
  void reset();

private:
  TeleopLimits limits_;
  Eigen::VectorXd low_passed_;
  bool seeded_ = false;
};

}  // namespace litearm_vr_teleop

#endif  // LITEARM_VR_TELEOP__TELEOP_MATH_HPP_
