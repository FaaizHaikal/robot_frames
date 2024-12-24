#include "robot_frames/model/robot_wrapper.hpp"

#include <fstream>
#include <iostream>

using namespace keisan::literals;

namespace robot_frames
{

RobotWrapper::RobotWrapper(const std::string & urdf_path)
{
  load_urdf(urdf_path);

  hip_pitch_offset = 30.0_deg;  // TODO: Get from config

  update_orientation(0.0_deg, 0.0_deg, 0.0_deg);
}

void RobotWrapper::load_urdf(const std::string & urdf_path)
{
  std::string urdf_xml = "";

  std::ifstream urdf_file(urdf_path, std::ios::in | std::ios::binary);
  if (urdf_file) {
    urdf_file.seekg(0, std::ios::end);
    urdf_xml.resize(urdf_file.tellg());
    urdf_file.seekg(0, std::ios::beg);
    urdf_file.read(&urdf_xml[0], urdf_xml.size());
    urdf_file.close();
  }

  urdf::Model model;
  KDL::Tree tree;

  if (!model.initString(urdf_xml)) {
    throw std::invalid_argument("Failed to initialize urdf model");
  }

  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    throw std::invalid_argument("Failed to extract kdl tree from urdf model");
  }

  joints.clear();
  fixed_joints.clear();
  add_joint(model, tree.getRootSegment());
}

void RobotWrapper::add_joint(
  const urdf::Model & model, const KDL::SegmentMap::const_iterator segment)
{
  const std::string & root_name = segment->second.segment.getName();

  std::vector<KDL::SegmentMap::const_iterator> children = segment->second.children;

  for (unsigned int i = 0; i < children.size(); i++) {
    const KDL::Segment & child = children[i]->second.segment;

    const std::string & child_name = children[i]->second.segment.getName();

    const std::string & joint_name = children[i]->second.segment.getJoint().getName();

    Joint joint(child, root_name, child_name);

    if (child.getJoint().getType() == KDL::Joint::None) {
      printf("Fixed link: %s -> %s\n", root_name.c_str(), child_name.c_str());
      fixed_joints.insert({joint_name, joint});
    } else {
      printf("Link: %s -> %s\n", root_name.c_str(), child_name.c_str());
      joints.insert({joint_name, joint});
    }

    add_joint(model, children[i]);
  }
}

void RobotWrapper::update_joint_position(
  const std::string & joint_name, keisan::Angle<double> position)
{
  auto joint = joints.find(joint_name);

  if (joint != joints.end()) {
    if (joint_name == "right_hip_pitch") {
      position -= hip_pitch_offset;
    } else if (joint_name == "left_hip_pitch") {
      position += hip_pitch_offset;
    }

    joint->second.position = position.radian();
  }
}

void RobotWrapper::update_orientation(
  keisan::Angle<double> roll, keisan::Angle<double> pitch, keisan::Angle<double> yaw)
{
  pitch += hip_pitch_offset;

  KDL::Rotation rotation = KDL::Rotation::RPY(roll.radian(), pitch.radian(), yaw.radian());

  double x, y, z, w;
  rotation.GetQuaternion(x, y, z, w);

  orientation.x = x;
  orientation.y = y;
  orientation.z = z;
  orientation.w = w;
}

}  // namespace robot_frames
