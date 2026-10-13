// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.
//
// The target viewer: the pose the operator's hand is asking for, drawn in RViz.
//
// Why this node exists
// --------------------
// When the arm does not follow, or follows somewhere unexpected, the question is
// where the chain broke: did the hand's motion reach ROS at all, did the
// increment mapping turn it into the pose the operator meant, or did the inverse
// kinematics give up? This node answers the first two of those with no robot, no
// controller and no motor in the loop. It reads the same frames the teleoperation
// node reads and draws the pose they ask for, so a target that moves while the
// arm does not puts the fault downstream of here.
//
// The same code, not the same arithmetic
// --------------------------------------
// The mapping is litearm_vr_teleop_math's TeleopIncrementMapper driven by
// singleArmUprightMapping(), which is the class vr_teleop_node runs; this file
// adds a viewpoint, not a second implementation of the maths. The anchor is the
// tip link's pose at the teleoperation start pose (forward kinematics of
// `start_pose`, falling back to kTeleopStartPose), and every headset frame since
// the first contributes its relative motion on top of that anchor.
//
// The mapping is velocity form, so the pose handed to feed() as the "current end
// effector pose" is this node's own previous target, seeded at the anchor. The
// increments then accumulate into "the start pose plus however far the hand has
// moved". vr_teleop_node hands feed() forward kinematics of the measured joints
// instead, which closes the mapping around where the arm actually is; this node
// has no arm to measure, so what it draws is the commanded trajectory dead
// reckoned from the anchor rather than the achieved one.
//
// The other visible difference from vr_teleop_node is the tcp_offset that node
// adds on top of the tip link: this viewer draws the tip link pose, because that
// is the pose the robot model shows, and a viewer that quietly drew a different
// point would be harder to hold against the model.
//
// What this node does not do
// --------------------------
// It reads no joint state, solves no inverse kinematics, and publishes nothing
// on any command topic. Its outputs are one tf frame, one Marker and one
// PoseStamped; it cannot move the arm.
//
// Seeing it in RViz
// -----------------
//   - a TF display, with `base_link` as the fixed frame, to see the `vr_target`
//     frame and its axes;
//   - a Marker display whose topic is /vr_target/marker (the display defaults to
//     /visualization_marker, so the topic has to be set);
//   - a Pose display on /vr_target/pose, which draws a single arrow.
// The marker carries the axis colours; the tf frame is what other displays can
// be attached to.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include "litearm_vr_teleop/msg/vr_controller_frame.hpp"
#include "litearm_vr_teleop/teleop_math.hpp"
#include "litearm_vr_teleop/teleop_planner.hpp"
#include "litearm_vr_teleop/teleop_start_pose.hpp"

namespace litearm_vr_teleop
{
namespace
{

/// The tf frame the target is published under, and the names of its two topics.
///
/// A frame of its own rather than something inside the robot's tree: `vr_target`
/// is not part of the machine, and adding it to the URDF so that it would show
/// up in the same list would be a lie about the robot.
constexpr char kTargetFrame[] = "vr_target";
constexpr char kMarkerTopic[] = "/vr_target/marker";
constexpr char kPoseTopic[] = "/vr_target/pose";

/// Namespace and fixed ids of the markers.
///
/// Fixed ids are what let the DELETE below name exactly the four markers this
/// node published, instead of clearing whatever else happens to be in the same
/// RViz display.
constexpr char kMarkerNamespace[] = "vr_target";
constexpr int32_t kOriginMarkerId = 3;

/// Size of the drawn axes, in metres.
///
/// 8 cm is roughly how far a hand travels in a fifth of a second: big enough to
/// be read at a glance across the arm's working volume, small enough not to be
/// mistaken for part of the robot.
constexpr double kAxisLength = 0.08;
constexpr double kShaftDiameter = 0.008;
constexpr double kHeadDiameter = 0.016;
constexpr double kOriginDiameter = 0.02;

/// X red, Y green, Z blue, the convention every axes display uses.
constexpr float kAxisColors[3][3] = {
  {1.0F, 0.0F, 0.0F},
  {0.0F, 1.0F, 0.0F},
  {0.0F, 0.0F, 1.0F},
};

/// Monotonic seconds, for the staleness window.
///
/// The same helper, for the same reason, as vr_teleop_node: a world clock would
/// make a simulated setup with a stepping /clock withdraw the marker at random.
double steadySeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

geometry_msgs::msg::Point toPointMessage(const Eigen::Vector3d & point)
{
  geometry_msgs::msg::Point message;
  message.x = point.x();
  message.y = point.y();
  message.z = point.z();
  return message;
}

/// The same three numbers as a Vector3, which is a different message type from
/// Point despite the identical shape; a transform's translation is one of these.
geometry_msgs::msg::Vector3 toVectorMessage(const Eigen::Vector3d & vector)
{
  geometry_msgs::msg::Vector3 message;
  message.x = vector.x();
  message.y = vector.y();
  message.z = vector.z();
  return message;
}

geometry_msgs::msg::Quaternion toQuaternionMessage(const Eigen::Quaterniond & quaternion)
{
  // Normalised on the way out: a quaternion that is a hair off the unit sphere
  // is not a rotation, and the transform would be rejected or silently wrong.
  const Eigen::Quaterniond normalised = quaternion.normalized();
  geometry_msgs::msg::Quaternion message;
  message.x = normalised.x();
  message.y = normalised.y();
  message.z = normalised.z();
  message.w = normalised.w();
  return message;
}

}  // namespace

class VrTargetVizNode : public rclcpp::Node
{
public:
  explicit VrTargetVizNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("vr_target_viz", options)
  {
    declareParameters();
  }

  /// Build the robot model, anchor on the start pose, and start publishing.
  ///
  /// Returns false when there is no anchor to draw from, which is either a model
  /// that could not be built or a start pose that does not fit it. A viewer that
  /// ran in that state would publish nothing, and nothing is indistinguishable
  /// from a headset that is not sending, so the reason is reported and the
  /// process stops instead.
  bool initialize()
  {
    mapper_ = std::make_unique<TeleopIncrementMapper>(mapping_, limits_);

    planner_ = std::make_unique<TeleopPlanner>(planner_options_, shared_from_this());
    std::string error;
    if (!planner_->initialize(error)) {
      RCLCPP_FATAL(
        get_logger(), "no robot model, so there is no pose to anchor the target to: %s",
        error.c_str());
      return false;
    }

    if (start_pose_.size() != static_cast<Eigen::Index>(planner_->dof())) {
      RCLCPP_FATAL(
        get_logger(),
        "start_pose has %ld entries while group '%s' has %zu joints; the anchor would be a pose "
        "this arm cannot be in",
        static_cast<long>(start_pose_.size()), group_name_.c_str(), planner_->dof());
      return false;
    }

    if (!planner_->forwardKinematics(start_pose_, anchor_position_, anchor_rotation_)) {
      RCLCPP_FATAL(
        get_logger(), "forward kinematics failed at the start pose: there is nothing to anchor to");
      return false;
    }

    // Transient local, on purpose. The durability of RViz's marker display is
    // editable, and it is the policy people change after markers fail to appear;
    // a transient local publisher is compatible with that setting AND with the
    // display's volatile default, while a volatile publisher is silently
    // incompatible with the first. Ten is the display's own depth.
    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      kMarkerTopic, rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());
    // Reliable and volatile, which is what RViz's Pose display asks for by
    // default; nothing on this topic needs the marker's durability trick.
    pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(kPoseTopic, rclcpp::QoS(10));

    // shared_from_this() rather than *this, because the broadcaster hands the
    // node straight to rclcpp::create_publisher for its /tf topic, and the shared
    // pointer is the form tf2_ros itself exercises.
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());

    // Best effort, ten deep. Best effort is the compatible direction whichever
    // way the headset link publishes: a best effort subscriber accepts a
    // reliable publisher as well as a best effort one, whereas a reliable
    // subscriber on a best effort publisher receives nothing at all. A dropped
    // frame costs this node a slightly staler drawing and nothing else, because
    // the increment mapping measures each frame against the last one it saw.
    frame_subscription_ = create_subscription<msg::VrControllerFrame>(
      controller_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      [this](const msg::VrControllerFrame::SharedPtr frame) {onFrame(frame);});

    const double rate = std::max(1.0, publish_rate_hz_);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {tick();});

    RCLCPP_INFO(
      get_logger(),
      "target viewer ready: frames on '%s', tf %s -> %s, marker '%s', pose '%s' at %.1f Hz",
      controller_topic_.c_str(), base_frame_.c_str(), kTargetFrame, kMarkerTopic, kPoseTopic, rate);
    RCLCPP_INFO(
      get_logger(),
      "anchored on the start pose [%.6f %.6f %.6f %.6f %.6f %.6f %.6f]: %s starts at "
      "[%.4f %.4f %.4f] m in %s",
      start_pose_(0), start_pose_(1), start_pose_(2), start_pose_(3), start_pose_(4),
      start_pose_(5), start_pose_(6), tip_link_.c_str(), anchor_position_.x(),
      anchor_position_.y(), anchor_position_.z(), base_frame_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "this node only draws: it publishes no command topic and cannot move the arm");
    return true;
  }

private:
  /// Read a parameter, declaring it only if it is not there yet.
  ///
  /// The guarded read vr_teleop_node uses, for the same reason: the node is
  /// created with `automatically_declare_parameters_from_overrides`, so a
  /// parameter that arrived from a launch file or a parameter file is already
  /// declared by the time the constructor runs, and a bare `declare_parameter`
  /// here would throw "already declared" for it.
  template <typename T>
  T readParameter(const std::string & name, const T & fallback)
  {
    if (has_parameter(name)) {
      return get_parameter(name).get_value<T>();
    }
    return declare_parameter<T>(name, fallback);
  }

  void declareParameters()
  {
    controller_topic_ = readParameter<std::string>("controller_topic", "/vr_controller_frame");
    base_frame_ = readParameter<std::string>("base_frame", "base_link");
    tip_link_ = readParameter<std::string>("tip_link", "ee_link");
    group_name_ = readParameter<std::string>("group_name", "litearm_arm");

    // Declared for the sake of their names. TeleopPlanner reads both values by
    // parameter name (see its options below), and a parameter has to be declared
    // before a launch file's override of it can reach the node. The documents
    // are not copied into members: they are whole URDF and SRDF strings.
    readParameter<std::string>("robot_description", std::string());
    readParameter<std::string>("robot_description_semantic", std::string());

    std::vector<double> start_pose(std::begin(kTeleopStartPose), std::end(kTeleopStartPose));
    start_pose = readParameter<std::vector<double>>("start_pose", start_pose);
    // An empty list stays an empty vector rather than being mapped: Eigen's
    // vector Map asserts when a zero size comes with a non-null pointer, and the
    // size check in initialize() reports the real problem anyway.
    if (!start_pose.empty()) {
      start_pose_ = Eigen::VectorXd::Map(
        start_pose.data(), static_cast<Eigen::Index>(start_pose.size()));
    }

    publish_rate_hz_ = readParameter<double>("publish_rate_hz", 30.0);
    timeout_s_ = readParameter<double>("timeout_s", 0.5);

    // The increment average is the one piece of the teleoperation node's mapping
    // tuning that changes what is DRAWN rather than how the joints are filtered,
    // so it is a parameter here too, with the same default as that node
    // (config/vr_teleop.yaml). With the same value the pose drawn is the pose
    // the teleoperation node computes.
    limits_.moving_average_samples = static_cast<std::size_t>(
      std::max(1, readParameter<int>("moving_average_samples", 10)));

    // The same control logic and its calibration as the teleoperation node, for
    // the same reason as the average above: a viewer that drew a different mapping
    // from the one the arm is being commanded with would send its user looking for
    // the difference in the arm.
    const bool tool_frame =
      readParameter<std::string>("control_frame", "base_link") == "tool";
    mapping_.rotation_mode = tool_frame ? RotationMapping::kTool : RotationMapping::kBaseLink;
    const std::vector<double> signs = readParameter<std::vector<double>>(
      tool_frame ? "tool_rotation_signs" : "base_link_rotation_signs",
      tool_frame ? std::vector<double>{1.0, -1.0, -1.0} : std::vector<double>{-1.0, -1.0, 1.0});
    if (signs.size() == 3u) {
      mapping_.rotation_signs = Eigen::Vector3d(signs[0], signs[1], signs[2]);
    }

    TeleopPlanner::Options planner_options;
    // Damped least squares, but only for its side effect: this node needs the
    // robot model and forward kinematics, both of which come from the MoveIt
    // core, and the dls backend is the one that loads nothing else. The plugin
    // backend would additionally need the solver's class and tuning at the
    // parameters of the group, which is a way for a viewer to fail on a machine
    // where the arm itself would run fine. The forward kinematics is the same
    // arithmetic either way.
    planner_options.backend = TeleopPlanner::IkBackend::kDls;
    planner_options.group_name = group_name_;
    planner_options.base_frame = base_frame_;
    planner_options.tip_link = tip_link_;
    planner_options.robot_description = "robot_description";
    planner_options.robot_description_semantic = "robot_description_semantic";
    planner_options_ = planner_options;
  }

  // ---------------------------------------------------------------- callbacks

  void onFrame(const msg::VrControllerFrame::SharedPtr frame)
  {
    // The same two filters as the teleoperation node, so that what is drawn is
    // what that node would act on: frames for the other hand and frames the
    // headset is not tracking are both ignored rather than drawn as motion.
    if (frame->side != "right") {
      RCLCPP_WARN_ONCE(
        get_logger(), "ignoring frames for side '%s'; this rig follows the right controller",
        frame->side.c_str());
      return;
    }
    if (!frame->tracked) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "the headset is not tracking the controller; ignoring frames");
      return;
    }

    const Eigen::Vector3d position(frame->position.x, frame->position.y, frame->position.z);
    const Eigen::Quaterniond orientation = Eigen::Quaterniond(
      frame->quaternion.w, frame->quaternion.x, frame->quaternion.y,
      frame->quaternion.z).normalized();

    if (!mapper_->active()) {
      // The first frame of a session, or the first one after the stream went
      // quiet. The anchoring frame produces no increment, so the target starts
      // out exactly at the anchor rather than jumping by however far the hand
      // travelled while nothing was arriving.
      mapper_->start(position, orientation);
      target_position_ = anchor_position_;
      target_rotation_ = anchor_rotation_;
      RCLCPP_INFO(
        get_logger(),
        "anchored on the controller: the target is the start pose plus the hand's motion from here");
    }

    // The mapping is velocity form: feed() returns the pose it is handed plus one
    // frame's worth of increment, so a target that accumulates the hand's motion
    // is built by feeding the PREVIOUS TARGET back in, seeded at the anchor.
    // Feeding the anchor every time instead would draw the last frame's
    // increment and nothing else, which is a few millimetres of jitter around the
    // start pose.
    //
    // vr_teleop_node feeds forward kinematics of the measured joints here rather
    // than its previous target, and that is the difference between the two
    // pictures: the arm's target is closed around where the arm actually is, so a
    // clipped or refused solution cannot accumulate, while this one is the
    // commanded trajectory dead reckoned from the anchor. Without a robot there
    // is no measured pose to close the loop with.
    const CartesianTarget target =
      mapper_->feed(position, orientation, target_position_, target_rotation_);
    if (!target.valid) {
      return;
    }

    target_position_ = target.position;
    target_rotation_ = target.rotation;
    have_target_ = true;
    marker_shown_ = true;
    last_frame_time_ = steadySeconds();
  }

  // ------------------------------------------------------------- publish loop

  void tick()
  {
    const rclcpp::Time stamp = get_clock()->now();

    if (marker_shown_ && steadySeconds() - last_frame_time_ > timeout_s_) {
      // The headset went quiet. The marker is withdrawn rather than left
      // standing where it was: a target that has stopped moving looks exactly
      // like an operator holding their hand still, and those two mean very
      // different things to whoever is reading the display.
      //
      // The tf frame and the pose keep the last target instead. A frame that
      // vanishes makes every display that refers to it report a missing
      // transform, and `vr_target` is exactly the kind of frame an operator
      // leaves selected; the marker is the part that would be read as "the hand
      // is here now".
      withdrawMarker(stamp);
      // Re-anchor on the next frame: after a gap, the controller pose is not
      // continuous with the pose before it, which is the same reason
      // vr_teleop_node resets its mapping on its own headset watchdog.
      mapper_->reset();
      RCLCPP_WARN(
        get_logger(),
        "no headset frame for %.2f s: the target marker is withdrawn. The tf frame keeps its last "
        "pose and the next frame re-anchors", timeout_s_);
    }

    if (!have_target_) {
      return;
    }
    publishTransform(stamp);
    publishPose(stamp);
    if (marker_shown_) {
      publishAxes(stamp, visualization_msgs::msg::Marker::ADD);
    }
  }

  void publishTransform(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = base_frame_;
    transform.child_frame_id = kTargetFrame;
    transform.transform.translation = toVectorMessage(target_position_);
    transform.transform.rotation = toQuaternionMessage(Eigen::Quaterniond(target_rotation_));
    tf_broadcaster_->sendTransform(transform);
  }

  void publishPose(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = base_frame_;
    message.pose.position = toPointMessage(target_position_);
    message.pose.orientation = toQuaternionMessage(Eigen::Quaterniond(target_rotation_));
    pose_publisher_->publish(message);
  }

  /// Remove the markers this node published, by naming their fixed ids.
  void withdrawMarker(const rclcpp::Time & stamp)
  {
    publishAxes(stamp, visualization_msgs::msg::Marker::DELETE);
    marker_shown_ = false;
  }

  /// Publish the three axis arrows and the origin sphere with one action.
  ///
  /// ADD every cycle, DELETE once when the headset goes quiet. Both draw from
  /// the same pose, so the withdrawn marker is always the one that was last
  /// shown.
  void publishAxes(const rclcpp::Time & stamp, int32_t action)
  {
    for (int axis = 0; axis < 3; ++axis) {
      // An arrow points along its own +x, so each axis is an arrow rotated to
      // the matching column of the target frame instead of a shape drawn along
      // the axis by hand.
      const Eigen::Vector3d direction = target_rotation_.col(axis);
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = base_frame_;
      marker.header.stamp = stamp;
      marker.ns = kMarkerNamespace;
      marker.id = axis;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = action;
      marker.pose.position = toPointMessage(target_position_);
      // FromTwoVectors picks the shortest rotation, which is all an arrow needs:
      // its roll about its own axis is not visible on a rotationally symmetric
      // shape.
      marker.pose.orientation = toQuaternionMessage(
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), direction));
      marker.scale.x = kAxisLength;
      marker.scale.y = kShaftDiameter;
      marker.scale.z = kHeadDiameter;
      marker.color.r = kAxisColors[axis][0];
      marker.color.g = kAxisColors[axis][1];
      marker.color.b = kAxisColors[axis][2];
      marker.color.a = 1.0F;
      // Not frame locked: a fresh pose arrives at the publishing rate, so there
      // is nothing for RViz to retransform between messages.
      marker.frame_locked = false;
      marker_publisher_->publish(marker);
    }

    // A sphere at the origin of the target, so that the position is still
    // readable when the axes happen to be seen end on.
    visualization_msgs::msg::Marker origin;
    origin.header.frame_id = base_frame_;
    origin.header.stamp = stamp;
    origin.ns = kMarkerNamespace;
    origin.id = kOriginMarkerId;
    origin.type = visualization_msgs::msg::Marker::SPHERE;
    origin.action = action;
    origin.pose.position = toPointMessage(target_position_);
    // An unset quaternion is all zeroes, which is not a rotation; the sphere is
    // symmetric but the message still has to be valid.
    origin.pose.orientation.w = 1.0;
    origin.scale.x = kOriginDiameter;
    origin.scale.y = kOriginDiameter;
    origin.scale.z = kOriginDiameter;
    // White, so that it reads as a position marker rather than as a fourth axis.
    origin.color.r = 1.0F;
    origin.color.g = 1.0F;
    origin.color.b = 1.0F;
    origin.color.a = 1.0F;
    origin.frame_locked = false;
    marker_publisher_->publish(origin);
  }

  // ------------------------------------------------------------------- members

  // Parameters.
  std::string controller_topic_;
  std::string base_frame_;
  std::string tip_link_;
  std::string group_name_;
  Eigen::VectorXd start_pose_;
  double publish_rate_hz_ = 30.0;
  double timeout_s_ = 0.5;
  TeleopLimits limits_;
  /// The measured baseline with this node's mapping parameters on top; see
  /// declareParameters.
  AxisMapping mapping_ = singleArmUprightMapping();
  TeleopPlanner::Options planner_options_;

  // Planning and mapping. Built in initialize().
  std::unique_ptr<TeleopPlanner> planner_;
  std::unique_ptr<TeleopIncrementMapper> mapper_;
  /// The end effector pose at the start pose, in base_frame_. Forward kinematics
  /// of the home joints, and the pose every increment is added to.
  Eigen::Vector3d anchor_position_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d anchor_rotation_ = Eigen::Matrix3d::Identity();

  // State.
  Eigen::Vector3d target_position_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d target_rotation_ = Eigen::Matrix3d::Identity();
  /// True once a target has been computed. Never cleared: the tf frame and the
  /// pose keep the last target when the headset goes quiet.
  bool have_target_ = false;
  /// True while the markers are in the display, i.e. while the stream is live.
  bool marker_shown_ = false;
  double last_frame_time_ = 0.0;

  // ROS interfaces. Built in initialize().
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<msg::VrControllerFrame>::SharedPtr frame_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace litearm_vr_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Overrides are declared automatically for the same reason vr_teleop_node does
  // it: the parameter reading in this node goes through the guarded helper
  // above, which expects a parameter that arrived from outside to already exist.
  // It also means a parameter file written for the whole stack can be handed to
  // this node without any of its entries being rejected as unknown.
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<litearm_vr_teleop::VrTargetVizNode>(options);
  if (!node->initialize()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
