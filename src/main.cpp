#include "robot_frames/robot_frames.hpp"

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::cerr << "Usage: ros2 run robot_frames <urdf_path> <walk_posture_path>" << std::endl;
    return 1;
  }

  std::string urdf_path = argv[1];
  std::string walk_posture_path = argv[3];

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("robot_frames");
  auto robot_frames_node =
    std::make_shared<robot_frames::RobotFramesNode>(node, urdf_path, walk_posture_path);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}