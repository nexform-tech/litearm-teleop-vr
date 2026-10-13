// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.
//
// The arithmetic a forward controller has to do on this arm, kept out of the
// realtime callback so it can be tested directly.

#ifndef LITEARM_VR_TELEOP__FORWARD_COMMAND_HPP_
#define LITEARM_VR_TELEOP__FORWARD_COMMAND_HPP_

#include <cstddef>

#include <Eigen/Core>

namespace litearm_vr_teleop
{

struct ForwardCommandLimits
{
  /// Joints the controller writes. Zero means "not configured yet".
  std::size_t dof = 0;
  /// Ceiling on the velocity written alongside each position, in rad/s.
  /// Zero or negative disables the ceiling, which is almost never wanted: the
  /// firmware has a speed limit of its own and reaching it is a fault.
  double max_velocity = 1.0;
  /// A command older than this is treated as absent and the joints hold. Zero
  /// or negative disables the staleness test.
  double command_timeout_s = 0.2;
};

/// Turns a stream of position setpoints into consistent position/velocity pairs.
///
/// This exists because of a property of this arm that is easy to miss and
/// expensive to discover on hardware: the firmware reads the velocity field of
/// the command frame as the slew ceiling of the position reference. A
/// controller that writes position alone leaves the velocity interface at
/// whatever the hardware initialised it to, which is zero, which freezes the
/// reference. The arm then accepts commands and does not move, with no error
/// reported anywhere. So every position this controller writes has to carry the
/// velocity that makes it reachable, and that velocity has to be derived from
/// the stream of setpoints itself, because the setpoints are the only thing
/// arriving.
class ForwardCommandTracker
{
public:
  explicit ForwardCommandTracker(ForwardCommandLimits limits);

  /// Seed the tracker at the measured pose.
  ///
  /// It has to be the real pose, not zero: the first command's velocity is
  /// derived from the distance between it and this pose, and taking that
  /// distance from a default would ask for a speed nobody requested.
  void reset(const Eigen::VectorXd & measured);

  /// Advance one cycle and write the result into caller owned buffers.
  ///
  /// Output is written through references rather than returned by value on
  /// purpose: this is called from a realtime callback, and returning a struct
  /// holding two vectors would allocate on every cycle.
  ///
  /// `target` is the current setpoint, or nullptr when there is none.
  /// `measured` is where the joints actually are.
  /// `command_age` is how long ago the setpoint arrived, in seconds; a negative
  /// value means no setpoint has ever arrived.
  ///
  /// `new_command` says whether this cycle is the one a setpoint arrived on.
  /// The velocity is worked out on that cycle and held until the next one,
  /// because there is nothing new to say in between: recomputing it every cycle
  /// puts zero in the field on every cycle without an arrival, and a field that
  /// is zero half the time is a slew ceiling of zero half the time. The firmware
  /// advances the reference at the ceiling, so that reads as an arm which moves
  /// about half as far as it was told to - and it is what a velocity plot of a
  /// teleoperation session shows as a band of spikes rather than a curve.
  ///
  /// `dt` is the interval the step from the previous setpoint to this one
  /// happened over, deliberately NOT the control cycle time. The velocity
  /// written alongside the position is the firmware's slew ceiling for the
  /// reference, so the honest value for it is the step divided by the time the
  /// operator took to make it. Dividing by the control cycle instead multiplies
  /// the request by the ratio of the two, and a command stream slower than the
  /// controller - the normal case, since the node solves inverse kinematics and
  /// the controller does not - turns every step into a saturated velocity
  /// request. See VrForwardController::update, which passes the measured
  /// interval between setpoint arrivals.
  ///
  /// On a hold the output is the MEASURED position, not the last setpoint. A
  /// hold is a statement about where the arm should be, and the honest answer is
  /// where it is; re-commanding a setpoint it may not have reached would keep
  /// asking for a move. The tracker's internal reference is synced to the
  /// measurement too, so the velocity of the next command is derived from
  /// reality rather than from a stale setpoint.
  ///
  /// Returns true when the output is a hold rather than a followed command.
  bool update(
    const Eigen::VectorXd * target, const Eigen::VectorXd & measured, double command_age,
    bool new_command, double dt, Eigen::VectorXd & positions, Eigen::VectorXd & velocities);

private:
  ForwardCommandLimits limits_;
  Eigen::VectorXd positions_;
  /// The velocity written on the last arrival, held in between.
  Eigen::VectorXd velocities_;
  bool seeded_ = false;
};

}  // namespace litearm_vr_teleop

#endif  // LITEARM_VR_TELEOP__FORWARD_COMMAND_HPP_
