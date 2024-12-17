#ifndef ROBOT_FRAMES__MODEL__ROBOT_WRAPPER_HPP_
#define ROBOT_FRAMES__MODEL__ROBOT_WRAPPER_HPP_

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <map>

namespace robot_frames
{

class SegmentPair
{
public:
  SegmentPair(const KDL::Segment & segment, const std::string & parent, const std::string & child)
  : segment(segment), parent(parent), child(child)
  {
  }

  KDL::Segment segment;
  std::string parent;
  std::string child;
};

class RobotWrapper
{
public:
  using MimicMap = std::map<std::string, urdf::JointMimic>;
  using LinkMap = std::map<std::string, SegmentPair>;

  const std::map<u_int8_t, std::string> joint_names = {
    {1, "right_shoulder_pitch"},
    {2, "left_shoulder_pitch"},
    {3, "right_shoulder_roll"},
    {4, "left_shoulder_roll"},
    {5, "right_elbow"},
    {6, "left_elbow"},
    {7, "right_hip_yaw"},
    {8, "left_hip_yaw"},
    {9, "right_hip_roll"},
    {10, "left_hip_roll"},
    {11, "right_hip_pitch"},
    {12, "left_hip_pitch"},
    {13, "right_knee"},
    {14, "left_knee"},
    {15, "right_ankle_pitch"},
    {16, "left_ankle_pitch"},
    {17, "right_ankle_roll"},
    {18, "left_ankle_roll"},
    {19, "neck_yaw"},
    {20, "neck_pitch"},
  };

  RobotWrapper(const std::string & urdf_path);

  void load_urdf(const std::string & urdf_path);
  void add_link(const urdf::Model & model, const KDL::SegmentMap::const_iterator segment);

  const MimicMap & get_mimics() const { return mimics; }
  const LinkMap & get_links() const { return links; }
  const LinkMap & get_fixed_links() const { return fixed_links; }

private:
  LinkMap links;
  LinkMap fixed_links;
  MimicMap mimics;
};

}  // namespace robot_frames

#endif  // ROBOT_FRAMES__MODEL__ROBOT_WRAPPER_HPP_
