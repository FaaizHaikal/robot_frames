#include "robot_frames/model/robot_wrapper.hpp"

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include "jitsuyo/config.hpp"

using namespace keisan::literals;

namespace robot_frames
{

RobotWrapper::RobotWrapper(
  const std::string & urdf_path, const std::string & camera_offset_path,
  const std::string & walk_posture_path)
{
  load_urdf(urdf_path);
  load_camera_offset(camera_offset_path);
  load_walk_posture(walk_posture_path);

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

  for (const auto & joint : model.joints_) {
    if (joint.second->mimic) {
      const auto & mimic = joint.second->mimic;
      mimic_joints.insert({mimic->joint_name, MimicJoint(joint.first, mimic->multiplier)});

      printf(
        "Joint %s is mimicking %s with multiplier %f\n", joint.first.c_str(),
        mimic->joint_name.c_str(), mimic->multiplier);
    }
  }

  joints.clear();
  fixed_joints.clear();
  add_joint(model, tree.getRootSegment());
}

void RobotWrapper::load_camera_offset(const std::string & camera_offset_path)
{
  nlohmann::json config;

  if (!jitsuyo::load_config(camera_offset_path, "camera_offset.json", config)) {
    throw std::runtime_error("Failed to load config file `camera_offset.json`");
  }

  double roll_double;
  double pitch_double;
  double yaw_double;

  bool valid_config = jitsuyo::assign_val(config, "x", camera_offset.position.x);
  valid_config &= jitsuyo::assign_val(config, "y", camera_offset.position.y);
  valid_config &= jitsuyo::assign_val(config, "z", camera_offset.position.z);
  valid_config &= jitsuyo::assign_val(config, "roll", roll_double);
  valid_config &= jitsuyo::assign_val(config, "pitch", pitch_double);
  valid_config &= jitsuyo::assign_val(config, "yaw", yaw_double);

  camera_offset.roll = keisan::make_degree(roll_double);
  camera_offset.pitch = keisan::make_degree(pitch_double);
  camera_offset.yaw = keisan::make_degree(yaw_double);

  if (!valid_config) {
    throw std::runtime_error("Failed to load config file `camera_offset.json`");
  }
}

void RobotWrapper::load_walk_posture(const std::string & walk_posture_path)
{
  nlohmann::json config;

  if (!jitsuyo::load_config(walk_posture_path, "kinematic.json", config)) {
    throw std::runtime_error("Failed to load config file `kinematic.json`");
  }

  bool valid_config = true;

  nlohmann::json offset_section;
  if (jitsuyo::assign_val(config, "offset", offset_section)) {
    bool valid_section = true;

    double hip_pitch_offset_double;
    double ankle_pitch_offset_double;
    double ankle_roll_offset_double;
    double ankle_yaw_offset_double;
    double x_offset_double;
    double y_offset_double;
    double z_offset_double;

    valid_section &=
      jitsuyo::assign_val(offset_section, "hip_pitch_offset", hip_pitch_offset_double);
    valid_section &= jitsuyo::assign_val(offset_section, "pitch_offset", ankle_pitch_offset_double);
    valid_section &= jitsuyo::assign_val(offset_section, "roll_offset", ankle_roll_offset_double);
    valid_section &= jitsuyo::assign_val(offset_section, "yaw_offset", ankle_yaw_offset_double);
    valid_section &= jitsuyo::assign_val(offset_section, "x_offset", x_offset_double);
    valid_section &= jitsuyo::assign_val(offset_section, "y_offset", y_offset_double);
    valid_section &= jitsuyo::assign_val(offset_section, "z_offset", z_offset_double);

    hip_pitch_offset = keisan::make_degree(hip_pitch_offset_double);
    ankle_pitch_offset = keisan::make_degree(ankle_pitch_offset_double);
    ankle_roll_offset = keisan::make_degree(ankle_roll_offset_double);
    ankle_yaw_offset = keisan::make_degree(ankle_yaw_offset_double);
    x_offset = keisan::make_degree(x_offset_double);
    y_offset = keisan::make_degree(y_offset_double);
    z_offset = keisan::make_degree(z_offset_double);

    if (!valid_section) {
      std::cerr << "Error found at section `offset`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  if (!valid_config) {
    throw std::runtime_error("Failed to load config file `kinematic.json`");
  }
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
      printf(
        "Fixed joint %s:  %s -> %s\n", joint_name.c_str(), root_name.c_str(), child_name.c_str());
      fixed_joints.insert({joint_name, joint});
    } else {
      printf("Joint %s: %s -> %s\n", joint_name.c_str(), root_name.c_str(), child_name.c_str());
      joints.insert({joint_name, joint});
    }

    add_joint(model, children[i]);
  }
}

void RobotWrapper::update_joint_position(
  const std::string & joint_name, keisan::Angle<double> position)
{
  auto joint = joints.find(joint_name);
  auto mimic_joint = mimic_joints.find(joint_name);

  if (joint != joints.end()) {
    joint->second.position = position.radian();

    if (mimic_joint != mimic_joints.end()) {
      auto joint = joints.find(mimic_joint->second.joint_name);
      if (joint != joints.end()) {
        joint->second.position = position.radian() * mimic_joint->second.multiplier;
      }
    }
  }
}

void RobotWrapper::set_camera_offset(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  camera_offset.position.x = x;
  camera_offset.position.y = y;
  camera_offset.position.z = z;
  camera_offset.roll = keisan::make_degree(roll);
  camera_offset.pitch = keisan::make_degree(pitch);
  camera_offset.yaw = keisan::make_degree(yaw);
}

void RobotWrapper::save_camera_offset()
{
  nlohmann::json config;

  config["x"] = camera_offset.position.x;
  config["y"] = camera_offset.position.y;
  config["z"] = camera_offset.position.z;
  config["roll"] = camera_offset.roll.degree();
  config["pitch"] = camera_offset.pitch.degree();
  config["yaw"] = camera_offset.yaw.degree();

  jitsuyo::save_config(camera_offset_path, "camera_offset.json", config);
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
