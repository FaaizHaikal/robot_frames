#include "robot_frames/node/robot_frames_node.hpp"

using namespace std::chrono_literals;

namespace robot_frames
{

RobotFramesNode::RobotFramesNode(
  const rclcpp::Node::SharedPtr & node, const std::string & urdf_path,
  const std::string & camera_offset_path, const std::string & walk_posture_path)
: node(node)
{
  robot_wrapper = std::make_shared<RobotWrapper>(urdf_path, camera_offset_path, walk_posture_path);

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

  get_camera_offset_service = node->create_service<GetCameraOffset>(
    "camera/get_camera_offset", [this](
                                  const GetCameraOffset::Request::SharedPtr request,
                                  GetCameraOffset::Response::SharedPtr response) {
      auto camera_offset = this->robot_wrapper->get_camera_offset();
      response->position_x = camera_offset.position.x;
      response->position_y = camera_offset.position.y;
      response->position_z = camera_offset.position.z;

      response->roll = camera_offset.roll.degree();
      response->pitch = camera_offset.pitch.degree();
      response->yaw = camera_offset.yaw.degree();

      response->ok = true;
    });

  update_camera_offset_service = node->create_service<UpdateCameraOffset>(
    "camera/update_camera_offset", [this](
                                     const UpdateCameraOffset::Request::SharedPtr request,
                                     UpdateCameraOffset::Response::SharedPtr response) {
      this->robot_wrapper->set_camera_offset(
        request->position_x, request->position_y, request->position_z, request->roll,
        request->pitch, request->yaw);

      if (request->save) {
        this->robot_wrapper->save_camera_offset();
      }

      response->ok = true;
    });

  publish_static_frames();

  node_timer = node->create_wall_timer(8ms, [this]() { publish_frames(); });
}

void RobotFramesNode::publish_frames()
{
  auto time = node->now();
  auto joints = robot_wrapper->get_joints();
  auto camera_offset = robot_wrapper->get_camera_offset();

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
    } else if (joint_name == "camera_joint") {
      // Apply camera offset
      tf_transform.transform.translation.x += camera_offset.position.x;
      tf_transform.transform.translation.y += camera_offset.position.y;
      tf_transform.transform.translation.z += camera_offset.position.z;

      tf2::Quaternion q_orig, q_offset, q_result;
      double roll, pitch, yaw;
      frame.M.GetRPY(roll, pitch, yaw);
      q_orig.setRPY(roll, pitch, yaw);
      q_offset.setRPY(
        camera_offset.roll.radian(), camera_offset.pitch.radian(), camera_offset.yaw.radian());
      q_result = q_offset * q_orig;
      q_result.normalize();

      tf_transform.transform.rotation.x = q_result.x();
      tf_transform.transform.rotation.y = q_result.y();
      tf_transform.transform.rotation.z = q_result.z();
      tf_transform.transform.rotation.w = q_result.w();
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
