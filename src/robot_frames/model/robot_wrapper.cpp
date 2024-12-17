#include "robot_frames/model/robot_wrapper.hpp"

#include <fstream>
#include <iostream>

namespace robot_frames
{

RobotWrapper::RobotWrapper(const std::string & urdf_path) { load_urdf(urdf_path); }

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

  mimics.clear();
  for (const auto & joint : model.joints_) {
    auto joint_mimic = urdf::JointMimic();
    if (joint.second->mimic) {
      joint_mimic.joint_name = joint.second->mimic->joint_name;
      joint_mimic.multiplier = joint.second->mimic->multiplier;
      joint_mimic.offset = joint.second->mimic->offset;

      mimics.insert({joint.first, joint_mimic});
    }
  }

  links.clear();
  fixed_links.clear();
  add_link(model, tree.getRootSegment());
}

void RobotWrapper::add_link(
  const urdf::Model & model, const KDL::SegmentMap::const_iterator segment)
{
  const std::string & root_name = segment->second.segment.getName();

  std::vector<KDL::SegmentMap::const_iterator> children = segment->second.children;

  for (unsigned int i = 0; i < children.size(); i++) {
    const KDL::Segment & child = children[i]->second.segment;

    const std::string & child_name = children[i]->second.segment.getName();

    const std::string & joint_name = children[i]->second.segment.getJoint().getName();

    SegmentPair pair(child, root_name, child_name);

    if (child.getJoint().getType() == KDL::Joint::None) {
      printf("Fixed link: %s -> %s\n", root_name.c_str(), child_name.c_str());
      fixed_links.insert({joint_name, pair});
    } else {
      printf("Link: %s -> %s\n", root_name.c_str(), child_name.c_str());
      links.insert({joint_name, pair});
    }

    // links.insert({joint_name, pair});

    add_link(model, children[i]);
  }
}

}  // namespace robot_frames
