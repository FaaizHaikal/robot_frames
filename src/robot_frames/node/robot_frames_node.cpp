#include "robot_frames/node/robot_frames_node.hpp"

using namespace std::chrono_literals;

namespace robot_frames
{

RobotFramesNode::RobotFramesNode(
  const rclcpp::Node::SharedPtr & node, const std::string & urdf_path,
  const std::string & walk_posture_path)
: node(node)
{
  robot_wrapper = std::make_shared<RobotWrapper>(urdf_path, walk_posture_path);

  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  current_joints_subscriber = node->create_subscription<CurrentJoints>(
    "joint/current_joints", 10, [this](const CurrentJoints::SharedPtr msg) {
      for (const auto & joint : msg->joints) {
        this->robot_wrapper->update_joint_position(
          this->robot_wrapper->joint_names.at(joint.id), keisan::make_degree(joint.position));
      }
    });

  kansei_status_subscriber = node->create_subscription<KanseiStatus>(
    "measurement/status", 10, [this](const KanseiStatus::SharedPtr msg) {
      auto roll = keisan::make_degree(msg->orientation.roll);
      auto pitch = keisan::make_degree(msg->orientation.pitch);
      auto yaw = keisan::make_degree(msg->orientation.yaw);

      this->robot_wrapper->update_orientation(roll, pitch, yaw);
    });

  publish_static_frames();

  node_timer = node->create_wall_timer(8ms, [this]() { publish_frames(); });
}

void RobotFramesNode::publish_frames()
{
  auto time = node->now();
  auto joints = robot_wrapper->get_joints();

  std::vector<TransformStamped> tf_transforms;

  for (const auto & joint : joints) {
    const std::string & joint_name = joint.first;
    const auto & joint_data = joint.second;

    TransformStamped tf_transform;

    tf_transform.header.stamp = time;
    tf_transform.header.frame_id = joint_data.parent;
    tf_transform.child_frame_id = joint_data.child;

    auto frame = joint_data.segment.pose(joint_data.position);

    tf_transform.transform.translation.x = frame.p.x();
    tf_transform.transform.translation.y = frame.p.y();
    tf_transform.transform.translation.z = frame.p.z();

    if (joint_name == "body_joint") {
      auto orientation = robot_wrapper->get_orientation();

      tf_transform.transform.rotation.x = orientation.x;
      tf_transform.transform.rotation.y = orientation.y;
      tf_transform.transform.rotation.z = orientation.z;
      tf_transform.transform.rotation.w = orientation.w;
    } else {
      frame.M.GetQuaternion(
        tf_transform.transform.rotation.x, tf_transform.transform.rotation.y,
        tf_transform.transform.rotation.z, tf_transform.transform.rotation.w);
    }

    tf_transforms.push_back(tf_transform);
  }

  tf_broadcaster->sendTransform(tf_transforms);
}

void RobotFramesNode::publish_static_frames()
{
  auto robot_joints = robot_wrapper->get_fixed_joints();
  std::vector<TransformStamped> tf_transforms;

  RCLCPP_INFO(node->get_logger(), "Publishing static frames");

  for (const auto & link : robot_joints) {
    KDL::Frame frame(link.second.segment.pose(0.0));

    TransformStamped tf_transform;

    tf_transform.header.stamp = node->now();
    tf_transform.header.frame_id = link.second.parent;
    tf_transform.child_frame_id = link.second.child;

    tf_transform.transform.translation.x = frame.p.x();
    tf_transform.transform.translation.y = frame.p.y();
    tf_transform.transform.translation.z = frame.p.z();

    frame.M.GetQuaternion(
      tf_transform.transform.rotation.x, tf_transform.transform.rotation.y,
      tf_transform.transform.rotation.z, tf_transform.transform.rotation.w);

    tf_transforms.push_back(tf_transform);
  }

  static_tf_broadcaster->sendTransform(tf_transforms);
}

}  // namespace robot_frames
