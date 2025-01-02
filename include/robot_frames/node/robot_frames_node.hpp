#ifndef ROBOT_FRAMES__NODE__ROBOT_FRAMES_NODE_HPP_
#define ROBOT_FRAMES__NODE__ROBOT_FRAMES_NODE_HPP_

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "kansei_interfaces/msg/status.hpp"
#include "robot_frames/model/robot_wrapper.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"

namespace robot_frames
{
class RobotFramesNode
{
public:
  using CurrentJoints = tachimawari_interfaces::msg::CurrentJoints;
  using KanseiStatus = kansei_interfaces::msg::Status;
  using TransformStamped = geometry_msgs::msg::TransformStamped;

  RobotFramesNode(
    const rclcpp::Node::SharedPtr & node, const std::string & urdf_path,
    const std::string & walk_posture_path);

  void publish_frames();
  void publish_static_frames();

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::TimerBase::SharedPtr node_timer;
  rclcpp::Time last_publish_time;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

  rclcpp::Subscription<CurrentJoints>::SharedPtr current_joints_subscriber;

  rclcpp::Subscription<KanseiStatus>::SharedPtr kansei_status_subscriber;

  std::shared_ptr<RobotWrapper> robot_wrapper;
};

}  // namespace robot_frames

#endif  // ROBOT_FRAMES__NODE__ROBOT_FRAMES_NODE_HPP_
