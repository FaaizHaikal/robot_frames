#ifndef ROBOT_FRAMES__MODEL__ROBOT_WRAPPER_HPP_
#define ROBOT_FRAMES__MODEL__ROBOT_WRAPPER_HPP_

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <map>

#include "keisan/angle.hpp"

using namespace keisan::literals;

namespace robot_frames
{
class RobotWrapper
{
public:
  struct Joint
  {
    KDL::Segment segment;
    std::string parent;
    std::string child;
    double position;

    Joint(
      const KDL::Segment & segment, const std::string & parent, const std::string & child,
      const double & position = 0)
    : segment(segment), parent(parent), child(child), position(position)
    {
    }
  };

  struct Orientation
  {
    double x;
    double y;
    double z;
    double w;
  };

  using JointMap = std::map<std::string, Joint>;

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

  RobotWrapper(const std::string & urdf_path, const std::string & walk_posture_path);

  void load_urdf(const std::string & urdf_path);
  void load_walk_posture(const std::string & walk_posture_path);

  void add_joint(const urdf::Model & model, const KDL::SegmentMap::const_iterator segment);
  void update_joint_position(const std::string & name, keisan::Angle<double> position);
  void update_orientation(
    keisan::Angle<double> roll, keisan::Angle<double> pitch, keisan::Angle<double> yaw);

  const JointMap & get_joints() const { return joints; }
  const JointMap & get_fixed_joints() const { return fixed_joints; }
  const Orientation & get_orientation() const { return orientation; }

private:
  JointMap joints;
  JointMap fixed_joints;
  Orientation orientation;

  keisan::Angle<double> hip_pitch_offset;
  keisan::Angle<double> ankle_pitch_offset;
  keisan::Angle<double> ankle_roll_offset;
  keisan::Angle<double> ankle_yaw_offset;
  keisan::Angle<double> x_offset;
  keisan::Angle<double> y_offset;
  keisan::Angle<double> z_offset;
};

}  // namespace robot_frames

#endif  // ROBOT_FRAMES__MODEL__ROBOT_WRAPPER_HPP_
