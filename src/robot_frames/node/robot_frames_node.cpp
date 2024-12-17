#include "robot_frames/node/robot_frames_node.hpp"

namespace robot_frames
{

RobotFramesNode::RobotFramesNode(
  const rclcpp::Node::SharedPtr & node, const std::string & urdf_path)
: node(node)
{
  robot_wrapper = std::make_shared<RobotWrapper>(urdf_path);

  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  current_joints_subscriber = node->create_subscription<CurrentJoints>(
    "joint/current_joints", 10,
    std::bind(&RobotFramesNode::update_joints, this, std::placeholders::_1));

  publish_static_frames();
}

void RobotFramesNode::update_joints(const CurrentJoints::SharedPtr msg)
{
  std::map<std::string, double> joint_positions;
  for (const auto & joint : msg->joints) {
    const std::string & joint_name = robot_wrapper->joint_names.at(joint.id);

    joint_positions.insert({joint_name, joint.position});
  }

  auto links = robot_wrapper->get_links();
  for (const auto & link : links) {
    const std::string & link_name = link.first;

    if (joint_positions.find(link_name) == joint_positions.end()) {
      joint_positions.insert({link_name, 0.0});
    }
  }

  auto mimics = robot_wrapper->get_mimics();

  for (const auto & mimic : mimics) {
    if (joint_positions.find(mimic.second.joint_name) != joint_positions.end()) {
      double position =
        joint_positions.at(mimic.second.joint_name) * mimic.second.multiplier + mimic.second.offset;

      joint_positions.insert({mimic.first, position});
    }
  }

  rclcpp::Time current_time = node->now();
  publish_frames(joint_positions, current_time);
}

void RobotFramesNode::publish_frames(
  const std::map<std::string, double> & joint_positions, const rclcpp::Time & time)
{
  std::vector<TransformStamped> tf_transforms;
  auto robot_links = robot_wrapper->get_links();

  RCLCPP_INFO(node->get_logger(), "Publishing frames");

  for (const auto & joint : joint_positions) {
    const std::string & joint_name = joint.first;
    const double & position = joint.second;

    auto link = robot_links.find(joint_name);

    if (link != robot_links.end()) {
      KDL::Frame frame(link->second.segment.pose(position));

      TransformStamped tf_transform;

      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = link->second.parent;
      tf_transform.child_frame_id = link->second.child;

      tf_transform.transform.translation.x = frame.p.x();
      tf_transform.transform.translation.y = frame.p.y();
      tf_transform.transform.translation.z = frame.p.z();

      frame.M.GetQuaternion(
        tf_transform.transform.rotation.x, tf_transform.transform.rotation.y,
        tf_transform.transform.rotation.z, tf_transform.transform.rotation.w);

      tf_transforms.push_back(tf_transform);
    } else {
      RCLCPP_WARN(node->get_logger(), "Joint %s not found in URDF", joint_name.c_str());
    }
  }

  tf_broadcaster->sendTransform(tf_transforms);
}

void RobotFramesNode::publish_static_frames()
{
  auto robot_links = robot_wrapper->get_fixed_links();
  std::vector<TransformStamped> tf_transforms;

  RCLCPP_INFO(node->get_logger(), "Publishing static frames");

  for (const auto & link : robot_links) {
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
