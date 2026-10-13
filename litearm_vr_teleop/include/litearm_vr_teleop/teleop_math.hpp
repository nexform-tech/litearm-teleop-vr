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
#include <vector>

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

/// Tick generator for the loop that advances the Cartesian target.
///
/// It exists so that the target advances on the node's clock rather than on the
/// headset's. Frames arrive when the link allows - about 72 Hz, with jitter, and
/// fewer when the WiFi is busy - so a target that stepped once per arrival would
/// be stepped by however long that particular arrival took: a late frame is a
/// double step, and the shorter the interval the smaller the step. Downstream
/// that is a velocity rather than a position, because the controller writes the
/// difference between two commands into the field the firmware reads as the slew
/// ceiling of its position reference, so the link's jitter becomes the arm's. A
/// tick here makes every step one control period long whatever the link did, and
/// leaves the headset's rate deciding only how fresh the pose in that step is.
///
/// The schedule is absolute - `t0 + n * period` - rather than a sleep of one
/// period per call, because a relative sleep adds this loop's own processing
/// time to the period it is trying to keep and drifts. A thread descheduled for
/// longer than a period gets one tick on its return, not the ticks it missed:
/// replaying them would ask the solver for the same target several times over,
/// and a burst of commands where the firmware wants a steady stream is the
/// defect this class exists to remove.
class ControlTick
{
public:
  /// A rate that cannot be honoured is repaired rather than trusted: one tick
  /// per second is the slowest schedule that still ticks, and a zero period
  /// would divide by zero.
  explicit ControlTick(double rate_hz);

  /// True when this call is on or past the next tick, and schedules the one
  /// after it.
  ///
  /// The first call is never a tick. Nothing has run yet, so there is no
  /// interval to have elapsed, and a tick at the moment the loop starts would
  /// only make the first period a fraction of the others.
  bool due(double now);

private:
  double period_;
  /// Origin of the grid, and how many periods have been handed out so far. The
  /// next tick is computed as `origin_ + ticks_ * period_` rather than
  /// accumulated by adding one period per tick: in floating point those additions
  /// do not land on the same numbers the clock uses (0.05 + 0.01 is not 0.06), so
  /// a caller asking at exactly the nominal tick time could land just before it,
  /// slip a whole poll interval, and - because the grid is absolute - never
  /// recover.
  double origin_ = 0.0;
  long long ticks_ = 0;
  bool started_ = false;
};

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
  /// Samples in the moving average of the Cartesian increment. Averaging before
  /// the inverse kinematics rather than after matters: it makes the target pose
  /// smooth, so the solver sees a consistent seed sequence instead of chasing
  /// noise.
  ///
  /// A sample is one control tick, because that is how often the node feeds this
  /// mapper: at the 100 Hz control rate this window is 100 ms, and it stays
  /// 100 ms whatever the headset is doing. It used to be one sample per headset
  /// frame, which made the same number mean 139 ms at the measured 72 Hz and
  /// 278 ms on a link that had dropped to 36 Hz - the smoothing stretched
  /// exactly when the link was worst, and the lag stretched with it. Ten samples
  /// was the measured setting; a rig that wants that same 139 ms of smoothing
  /// sets 14.
  std::size_t moving_average_samples = 10;
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

/// Cartesian velocity of the tool, in the robot base frame.
///
/// The velocity form of CartesianTarget, and the form MoveIt Servo works in. The
/// difference is not cosmetic: a hand moving at a steady rate produces a steady
/// twist, and a twist is what the arm's Jacobian answers directly, so the joint
/// command is a speed that is integrated against the node's own clock rather
/// than a position the joints have to chase. Nothing accumulates behind an arm
/// that lags, because there is no target ahead of it to accumulate.
///
/// The angular part is a rotation vector rate, so it composes the same way the
/// mapper's rotation increments do.
struct CartesianTwist
{
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();   ///< m/s, robot base frame
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();  ///< rad/s, robot base frame
  /// False until the first frame after `start()`, and while paused.
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
/// in: with the reference on the measured pose, the target is re-based on the
/// measured pose every tick, so an increment below the deadband is not
/// accumulated, it is dropped. A deadband expressed in metres therefore acts as
/// a velocity threshold of `deadband / tick period` (0.5 mm at 100 Hz is
/// 5 cm/s). That is why the default is zero: with the reference on the commanded
/// pose the increments do accumulate and a deadband is safe, with the reference
/// on the measured pose a large one would be the very "nothing happens until I
/// move my hand a long way" the deadband is meant to avoid.
///
/// The threshold is a number of centimetres per second rather than a number of
/// metres per frame, which is a consequence of ticking the target on the node's
/// clock: the same deadband used to mean 3.6 cm/s at the headset's 72 Hz and
/// half that on a slower link.
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

  /// Feed one headset frame and produce the tool twist it commands.
  ///
  /// The velocity form of `feed`, and the one the velocity following mode uses.
  /// `dt` is the time since the previous call - the caller's tick, not the
  /// headset's frame period - so a frame that has not been replaced since the
  /// last call contributes no motion, which is what makes this idempotent under a
  /// caller running faster than the headset reports. The window averages RATES
  /// over the time the samples really covered, so a late tick produces the same
  /// average speed as an on-time one, where an average of increments would
  /// produce a smaller one.
  CartesianTwist feedRate(
    const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation, double dt);

private:
  /// Turn a headset pose into a homogeneous transform.
  static Eigen::Matrix4d poseToHomogeneous(
    const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation);

  /// The controller's motion since the previous frame, carried into the robot
  /// frame: translation in metres and rotation as a rotation vector, as one
  /// six element vector. Shared by both output forms, because they differ only
  /// in what they do with it.
  Eigen::Matrix<double, 6, 1> relativeMotion(const Eigen::Matrix4d & controller);

  /// One averaged entry of the window, with the interval it covered. The
  /// interval is kept because the two output forms disagree about what to do
  /// with a late sample: the increment form adds it as it is, the rate form
  /// divides by the time it took.
  struct Sample
  {
    Eigen::Matrix<double, 6, 1> motion = Eigen::Matrix<double, 6, 1>::Zero();
    double dt = 0.0;
  };

  AxisMapping mapping_;
  TeleopLimits limits_;
  bool active_ = false;
  Eigen::Matrix4d previous_controller_ = Eigen::Matrix4d::Identity();
  std::deque<Sample> history_;
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

/// Ceilings on how fast the commanded joints may change, and how fast that
/// change may itself change.
///
/// These are the numbers a motion profile is made of, and they exist because of
/// what the firmware does with the velocity the controller writes: it reads that
/// field as the slew ceiling of the position reference. A command stream whose
/// velocity steps - which is what a joint target that jumps by a few
/// hundredths of a radian between two commands produces - therefore asks the
/// firmware for a step in speed, and the field ends up pinned at its ceiling
/// instead of describing the motion. Bounding the velocity, then the
/// acceleration, then the jerk turns that stream into a curve the firmware can
/// follow, at the cost of the arm taking a little longer to reach a new target.
///
/// The defaults are a rated joint speed of 2 rad/s (the arm's own figure, the
/// same one `TeleopLimits::max_joint_velocity` carries), reaching it in 0.1 s,
/// and reaching full acceleration in 20 ms. The jerk ceiling is high because it
/// is there to soften the ends of a move, not to make the arm slow: it is what
/// stops the velocity from stepping, and a ceiling that also delays the
/// deceleration is a ceiling that makes the arm overshoot. They are a starting
/// point to tune against what the arm actually does, not measurements.
struct JointRateLimits
{
  /// Ceiling on the joint speed, in rad/s. Same meaning as
  /// `TeleopLimits::max_joint_velocity`, and the limiter takes over that job
  /// when it is in the chain.
  double max_velocity = 2.0;
  /// Ceiling on the joint acceleration, in rad/s^2. A step in velocity is spread
  /// over `step / max_acceleration` seconds.
  double max_acceleration = 20.0;
  /// Ceiling on the rate of change of the acceleration, in rad/s^3. This is what
  /// makes the velocity curve an S rather than a ramp: the acceleration cannot
  /// jump from zero to its ceiling either, so a motion starts and ends softly.
  double max_jerk = 1000.0;
};

/// Turns a stream of joint targets into a command whose speed, acceleration and
/// jerk are all bounded.
///
/// The limiter carries two states per joint - the speed it is asking for and the
/// acceleration it is using - and works out, every cycle, the fastest speed the
/// joint may hold and still stop on its target. That speed comes from the
/// stopping law `v^2 / (2 a) + v a / (2 j) <= s`, solved for the speed rather
/// than for the distance, with one more term for the time the loop and the
/// acceleration ramp take. The speed state is then chased towards it by a first
/// order loop, and the result is limited twice: the speed may not change faster
/// than `max_acceleration`, and that acceleration may not change faster than
/// `max_jerk`.
///
/// Working out a speed limit rather than deciding when to start braking is the
/// point of the design. A decision has to be latched or it chatters, and a
/// latched brake holds full deceleration until it releases, which stalls the
/// joint in the middle of a move. A limit is a smooth function of the distance,
/// so there is nothing to switch and nothing to chatter.
///
/// Three properties make it safe in front of an operator. It never steps past
/// the target: the step it returns is clipped to the remaining distance, so a
/// limiter that has fallen behind stops where it is asked to instead of
/// oscillating around the target. It has unit gain at steady state, because the
/// limit includes the target's own rate as a feed forward - a target moving at a
/// constant rate is held at that rate with no distance left over, which is the
/// property the joint filter below it was written to preserve. And it never asks
/// to move away from the target, so a joint that has arrived stays arrived.
class JointRateLimiter
{
public:
  JointRateLimiter(JointRateLimits limits, std::size_t dof);

  /// Returns the joint values to command for this cycle.
  ///
  /// `target` is where the joints should end up (the inverse kinematics result,
  /// already filtered), `current` is the measured joint state, and `dt` is this
  /// cycle's interval. A zero or negative `dt` is treated as the smallest
  /// interval worth dividing by, so a starved loop cannot authorise a step.
  Eigen::VectorXd apply(
    const Eigen::VectorXd & target, const Eigen::VectorXd & current, double dt);

  /// Forget the velocity and acceleration. The next call starts from rest, so
  /// resuming after a pause does not begin with a step, and does not inherit a
  /// velocity the operator asked for minutes ago.
  void reset();

  /// The speed the limiter is asking for, per joint, in rad/s.
  ///
  /// This is the value to publish beside the position, because on this arm the
  /// controller writes it into a field the firmware reads as the slew ceiling of
  /// its position reference. Taking it from here rather than differentiating the
  /// published positions is what puts the ceiling on the control clock: a
  /// difference of two positions divided by the interval between their arrivals
  /// carries the arrival jitter, which at 100 Hz is most of the interval itself,
  /// and a ceiling that jitters is a reference that moves at a jittering speed.
  ///
  /// It is the speed BEFORE the arrival guard clips the last step, which is
  /// deliberate: a ceiling bounds how fast the reference may travel, and the
  /// guard's trim is about not stepping past the target.
  const Eigen::VectorXd & velocity() const { return velocity_; }

private:
  JointRateLimits limits_;
  /// How hard the speed loop chases the limit curve, in 1/s. At 50 the loop
  /// settles in about 20 ms, which is fast enough to follow the curve closely
  /// and slow enough to be stable at a 10 ms cycle.
  double speed_gain_ = 50.0;
  Eigen::VectorXd velocity_;
  Eigen::VectorXd acceleration_;
  /// The target the previous cycle was given, so the limiter can tell a target
  /// that is moving from one that has stopped.
  Eigen::VectorXd previous_target_;
  bool seeded_ = false;
};

/// Ceilings and smoothing for a joint velocity command.
///
/// The velocity following mode's counterpart of JointRateLimits, and the three
/// numbers MoveIt Servo reaches its smooth motion with: the joint's own velocity
/// and acceleration limits, and one smoothing constant. Servo carries that
/// constant as `low_pass_filter_coeff`, a per iteration coefficient; a time
/// constant is used here because this loop measures its interval rather than
/// assuming one, and a coefficient would mean a different cutoff every time a
/// tick ran late - which is the class of bug this whole file keeps tripping over.
struct JointVelocityLimits
{
  /// Ceiling on the joint speed, rad/s.
  double max_velocity = 2.0;
  /// Ceiling on how fast that speed may change, rad/s^2.
  double max_acceleration = 20.0;
  /// Time constant of the low pass on the commanded speed, seconds. Zero
  /// disables the smoothing, which leaves the acceleration ceiling to shape the
  /// command on its own.
  double filter_tau = 0.03;
};

/// Smooths a joint velocity command and bounds it.
///
/// This is the class that makes the velocity following mode smooth, and the
/// reason it exists instead of JointRateLimiter is what each one is handed. A
/// rate limiter is given a position and has to decide how fast to approach it;
/// this is given the speed itself, because the twist and the Jacobian produced
/// it, so the filter acts directly on the quantity the hardware uses. On this arm
/// that quantity is also the one the firmware reads as the slew ceiling of its
/// position reference, which is why publishing it as it is matters: nothing
/// downstream has to differentiate two positions to guess a speed, and a
/// derivative of noisy positions is a noisy speed.
class JointVelocityProfile
{
public:
  JointVelocityProfile(JointVelocityLimits limits, std::size_t dof);

  /// The velocity to command this cycle.
  ///
  /// `target` is the speed the twist asked for and `dt` is this cycle's
  /// interval. A zero or negative interval is treated as the smallest worth
  /// integrating over, so a starved loop cannot authorise a step.
  Eigen::VectorXd apply(const Eigen::VectorXd & target, double dt);

  /// Forget the filter state. The next call starts from rest, so resuming after
  /// a pause does not begin at a speed the operator asked for minutes ago.
  void reset();

  /// The speed this class is asking for, which is what the controller writes
  /// beside the position.
  const Eigen::VectorXd & velocity() const { return velocity_; }

private:
  JointVelocityLimits limits_;
  Eigen::VectorXd velocity_;
};

/// Integrate one cycle of a joint velocity command, inside the joint range.
///
/// This is the step that turns a speed into the position the controller writes
/// beside it, and doing it here - on the node's clock, at the node's rate - is
/// what keeps the position reference the node's rather than the link's.
///
/// A joint at its limit stops there instead of running past it: the hardware
/// would refuse the motion anyway, and a command that kept climbing outside the
/// range would only build a difference that has to be given back the moment the
/// operator reverses.
Eigen::VectorXd integrateJointVelocity(
  const Eigen::VectorXd & position, const Eigen::VectorXd & velocity, double dt,
  const Eigen::VectorXd & lower, const Eigen::VectorXd & upper);

/// Whether the trigger counts as pressed, with hysteresis.
///
/// One threshold on an analogue input that rests anywhere near it is an
/// oscillator: a hand holding the trigger half way, a worn pot, or a headset
/// reporting a value that drifts across the line all produce a stream of press and
/// release edges, and the debounce cannot help - it only asks the signal to stay
/// put for 50 ms, which a slow wobble does. Measured on this rig: an open/close
/// pair every half second, at exactly the minimum close window, from a trigger
/// nobody was touching.
///
/// Hysteresis is the answer rather than a longer debounce, because a longer
/// debounce delays every real press while a pair of thresholds costs nothing: it
/// takes `press_at` to close and it takes a release below
/// `press_at - hysteresis` to open, so a signal that wanders between them keeps
/// whatever state it already had.
bool triggerPressed(double trigger, bool was_pressed, double press_at, double hysteresis);

/// Scale a joint velocity command down as the reference runs away from the arm.
///
/// The velocity form integrates, so an arm that lags leaves the reference further
/// ahead of it every cycle, and an operator who keeps pushing while the arm
/// cannot follow winds the reference out until a joint limit stops it and the
/// arm then holds there. The position form has the same guard stated in Cartesian
/// terms (`max_target_lead_m` and `max_target_lead_rad`, which pull a target that
/// has run ahead back onto the sphere of that radius); this is it in joint terms,
/// where the integration actually happens, and it scales the WHOLE command rather
/// than clamping joints one at a time, so the direction the operator asked for
/// survives the scaling. MoveIt Servo reaches the same quantity from its speed
/// scaling factor.
///
/// `error` is the reference minus the measurement, one per joint, and `margin` is
/// how much lead is allowed. The result is 1 while every joint is inside a
/// fraction of the margin and falls linearly to zero at the margin, which keeps
/// the scaling continuous: a threshold with a jump in it is a chatter source, and
/// this one multiplies the whole command.
double leadScale(const Eigen::VectorXd & error, double margin);

/// Re-express a twist measured at a point offset from the frame it is used in.
///
/// A body turning at `angular` carries a point at `offset` - measured from the
/// origin of the frame, expressed in that frame - faster by `angular x offset`,
/// so the same rigid motion seen at the origin is this twist minus that term.
/// The two points are not interchangeable on this arm: the operator's hand moves
/// the tool tip, the node's Jacobian is the arm's Jacobian at the end effector
/// link, and the distance between them is `tcp_offset`.
CartesianTwist twistAtOffset(const CartesianTwist & twist, const Eigen::Vector3d & offset);

}  // namespace litearm_vr_teleop

#endif  // LITEARM_VR_TELEOP__TELEOP_MATH_HPP_
