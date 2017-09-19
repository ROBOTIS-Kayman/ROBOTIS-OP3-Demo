/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Kayman Jung */

#include "robotis_op3_following_motion/op3_motion_follower.h"

namespace robotis_op
{
MotionFollower::MotionFollower()
  : nh_(ros::this_node::getName()),
    MODULE_NAME("direct_control_module"),
    l_shoulder_3d_(0, 0, 0),
    is_ready_(false),
    DEBUG_PRINT(false)
{
  human_pose_sub_ = nh_.subscribe("/openpose/pose", 1, &MotionFollower::humanPoseCallback, this);
  button_sub_ = nh_.subscribe("/robotis/open_cr/button", 1, &MotionFollower::buttonCallback, this);
  op3_joints_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/direct_control/set_joint_states", 0);
  dxl_torque_pub_ = nh_.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  set_module_pub_ = nh_.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);

  std_msgs::String module_name;
  module_name.data = "direct_control_module";
  set_module_pub_.publish(module_name);

  joint_angles_.clear();
}

MotionFollower::~MotionFollower()
{

}

void MotionFollower::humanPoseCallback(const openpose_ros_msgs::Persons::ConstPtr& msg)
{
  if(is_ready_ == false)
    return;

  // check human pose data
  if(msg->persons.size() == 0)
    return;

  ROS_INFO_STREAM("Pose message!! - " << msg->persons.size());
  //  person_to_follow_ = msg->persons.front();

  // calc joint angle of ROBOTIS-OP3 to follow the motion of person who is recognized at first.
  calcJointStates(msg->persons.front());

  // publish joint angle
  publishJointStates();
}

void MotionFollower::buttonCallback(const std_msgs::String::ConstPtr& msg)
{
  // if user button is pressed, OP3 will torque on and go init pose
  if(msg->data == "user")
  {
    // check torque
    checkTorque();

    // go init pose and start following demo
    goInitPose();
  }
}

void MotionFollower::checkTorque()
{
  std_msgs::String check_msg;
  check_msg.data = "check";

  // send command to op3_manager in order to check the torque condition of OP3
  dxl_torque_pub_.publish(check_msg);
}

void MotionFollower::goInitPose()
{
  // set module to direct_control_module
  setModule("none");

  usleep(20 * 1000);

  is_ready_ = false;

  setModule(MODULE_NAME);

  usleep(20 * 1000);

  // go init pose from yaml file
  parseInit();

  publishJointStates();

  usleep(2.5 * 1000 * 1000);

  is_ready_ = true;
}

void MotionFollower::setModule(const std::string &module_name)
{
  // set module to direct_control_module for this demonsration
  std_msgs::String module_msg;
  module_msg.data = module_name;

  set_module_pub_.publish(module_msg);
}

void MotionFollower::parseInit()
{
  joint_angles_.clear();
  std::string init_pose_path = ros::package::getPath("robotis_op3_following_motion") + "/data/ini_pose.yaml";

  if(init_pose_path == "")
    return;

  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(init_pose_path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator yaml_it = tar_pose_node.begin(); yaml_it != tar_pose_node.end(); ++yaml_it)
  {
    std::string joint_name;
    double value;

    joint_name = yaml_it->first.as<std::string>();
    value = yaml_it->second.as<double>();

    joint_angles_[joint_name] = value * M_PI / 180.0;
  }
}

void MotionFollower::calcJointStates(const openpose_ros_msgs::PersonDetection &person_to_follow)
{
  joint_angles_.clear();

  ROS_INFO("Get person data");
  // store body position
  for(int ix = 0; ix < person_to_follow.body_part.size(); ix++)
  {
    openpose_ros_msgs::BodyPartDetection body_part = person_to_follow.body_part[ix];
    body_position_[body_part.part_id] = Eigen::Vector2d(body_part.x, body_part.y);
  }

  bool calc_result = false;

  // calc left shoulder
  double neck_to_shoulder_ratio = 11.0 / 7.0;

  if(body_position_[Neck] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[LShoulder] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[LElbow] != Eigen::Vector2d(0.0, 0.0))
  {
    double l_shoulder_roll = 0.0, l_shoulder_pitch = 0.0;

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Neck : (" << body_position_[Neck].coeff(0) << ", " << body_position_[Neck].coeff(1)
                         << "), LShoulder : (" << body_position_[LShoulder].coeff(0) << ", " << body_position_[LShoulder].coeff(1)
                         << "), LElbow : (" << body_position_[LElbow].coeff(0) << ", " << body_position_[LElbow].coeff(1) << ")");

    Eigen::Vector2d neck_vector = body_position_[Neck] - body_position_[LShoulder];
    Eigen::Vector2d shoulder_vector = body_position_[LShoulder] - body_position_[LElbow];
    Eigen::Vector2d arm_vector = body_position_[Neck] - body_position_[LElbow];

    double neck_len = neck_vector.norm();
    double shoulder_len_target = neck_len * neck_to_shoulder_ratio;
    double shoulder_len = shoulder_vector.norm();
    if(shoulder_len > shoulder_len_target) shoulder_len_target = shoulder_len;

    ROS_INFO_COND(DEBUG_PRINT, "==============================================");
    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Neck length : " << neck_len << ", Shoulder length : " << shoulder_len);

    double l_shoulder_angle = acos(shoulder_len / shoulder_len_target);
    Eigen::Vector3d should_3d_vec = Eigen::Vector3d(sin(l_shoulder_angle) * shoulder_len_target, shoulder_vector[0], shoulder_vector[1]);


    double alpha = 0.75;
    if(l_shoulder_3d_ == Eigen::Vector3d(0, 0, 0)) alpha = 0;
    l_shoulder_3d_ = l_shoulder_3d_ * alpha + should_3d_vec * (1 - alpha);

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Shoulder 3D Vector : [alpha - " << alpha << "]\n" << l_shoulder_3d_);

    Eigen::Vector3d toward(0, 1, 0);
    should_3d_vec.normalize();
    Eigen::Quaterniond shoulder_orientation(Eigen::Quaterniond::FromTwoVectors(toward, should_3d_vec));
    Eigen::Vector3d shoulder_rpy = robotis_framework::convertRotationToRPY(shoulder_orientation.toRotationMatrix());

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Shoulder Roll : " << (shoulder_rpy[0] * 180.0 / M_PI) << ", Pitch : " << (shoulder_rpy[1] * 180.0 / M_PI) << ", Yaw : "  << (shoulder_rpy[2] * 180.0 / M_PI));

    // adjustment shoulder


    // get a shoulder pitch angle
    calc_result = calcJointAngle(neck_vector, shoulder_vector, l_shoulder_roll);

    Eigen::Vector3d shoulder_vector_3d, arm_vector_3d;
    shoulder_vector_3d << shoulder_vector , 0;
    arm_vector_3d << arm_vector, 0;
    Eigen::Vector3d shoulder_direction = shoulder_vector_3d.cross(arm_vector_3d);

    if(shoulder_direction.coeff(2) > 0)
      l_shoulder_roll *= (-1);

    //if(calc_result == true)
    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Left_Shoulder: [Roll] " << (l_shoulder_roll * 180 / M_PI) << ", [Pitch] " << (l_shoulder_pitch * 180 / M_PI));

    checkMaxAngle(80 * M_PI / 180.0, l_shoulder_roll);
    joint_angles_["l_sho_pitch"] = l_shoulder_pitch;
    joint_angles_["l_sho_roll"] = l_shoulder_roll;
  }
  else
  {
    l_shoulder_3d_ = Eigen::Vector3d(0, 0, 0);
  }

  // calc right shoulder
  if(body_position_[Neck] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[RShoulder] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[RElbow] != Eigen::Vector2d(0.0, 0.0))
  {
    double r_shoulder_roll = 0.0, r_shoulder_pitch = 0.0;

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Neck : (" << body_position_[Neck].coeff(0) << ", " << body_position_[Neck].coeff(1)
                         << "), LShoulder : (" << body_position_[RShoulder].coeff(0) << ", " << body_position_[RShoulder].coeff(1)
                         << "), LElbow : (" << body_position_[RElbow].coeff(0) << ", " << body_position_[RElbow].coeff(1) << ")");

    Eigen::Vector2d neck_vector = body_position_[Neck] - body_position_[RShoulder];
    Eigen::Vector2d shoulder_vector = body_position_[RShoulder] - body_position_[RElbow];
    Eigen::Vector2d arm_vector = body_position_[Neck] - body_position_[RElbow];

    double neck_len = neck_vector.norm();
    double shoulder_len_target = neck_len * neck_to_shoulder_ratio;
    double shoulder_len = shoulder_vector.norm();
    if(shoulder_len > shoulder_len_target) shoulder_len_target = shoulder_len;

    ROS_INFO_COND(DEBUG_PRINT, "==============================================");
    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Neck length : " << neck_len << ", Shoulder length : " << shoulder_len);

    double r_shoulder_angle = acos(shoulder_len / shoulder_len_target);
    Eigen::Vector3d should_3d_vec = Eigen::Vector3d(sin(r_shoulder_angle) * shoulder_len_target, shoulder_vector[0], shoulder_vector[1]);


    double alpha = 0.75;
    if(r_shoulder_3d_ == Eigen::Vector3d(0, 0, 0)) alpha = 0;
    r_shoulder_3d_ = r_shoulder_3d_ * alpha + should_3d_vec * (1 - alpha);

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Shoulder 3D Vector : [alpha - " << alpha << "]\n" << r_shoulder_3d_);

    Eigen::Vector3d toward(0, 1, 0);
    should_3d_vec.normalize();
    Eigen::Quaterniond shoulder_orientation(Eigen::Quaterniond::FromTwoVectors(toward, should_3d_vec));
    Eigen::Vector3d shoulder_rpy = robotis_framework::convertRotationToRPY(shoulder_orientation.toRotationMatrix());

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Shoulder Roll : " << (shoulder_rpy[0] * 180.0 / M_PI) << ", Pitch : " << (shoulder_rpy[1] * 180.0 / M_PI) << ", Yaw : "  << (shoulder_rpy[2] * 180.0 / M_PI));

    // adjustment shoulder


    // get a shoulder pitch angle
    calc_result = calcJointAngle(neck_vector, shoulder_vector, r_shoulder_roll);

    Eigen::Vector3d shoulder_vector_3d, arm_vector_3d;
    shoulder_vector_3d << shoulder_vector , 0;
    arm_vector_3d << arm_vector, 0;
    Eigen::Vector3d shoulder_direction = shoulder_vector_3d.cross(arm_vector_3d);

    if(shoulder_direction.coeff(2) > 0)
      r_shoulder_roll *= (-1);

    //if(calc_result == true)
    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Left_Shoulder: [Roll] " << (r_shoulder_roll * 180 / M_PI) << ", [Pitch] " << (r_shoulder_pitch * 180 / M_PI));

    checkMaxAngle(80 * M_PI / 180.0, r_shoulder_roll);
    joint_angles_["r_sho_pitch"] = r_shoulder_pitch;
    joint_angles_["r_sho_roll"] = r_shoulder_roll;
  }
  else
  {
    r_shoulder_3d_ = Eigen::Vector3d(0, 0, 0);
  }

  // calc left arm

  if(body_position_[LShoulder] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[LElbow] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[LWrist] != Eigen::Vector2d(0.0, 0.0))
  {
    double l_elbow = 0.0;
    bool is_upside_left = false;

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "LShoulder : (" << body_position_[LShoulder].coeff(0) << ", " << body_position_[LShoulder].coeff(1)
                         << "), LElbow : (" << body_position_[LElbow].coeff(0) << ", " << body_position_[LElbow].coeff(1)
                         << "), LWrist : (" << body_position_[LWrist].coeff(0) << ", " << body_position_[LWrist].coeff(1) << ")");

    Eigen::Vector2d shoulder_vector = body_position_[LShoulder] - body_position_[LElbow];
    Eigen::Vector2d elbow_vector = body_position_[LElbow] - body_position_[LWrist];
    Eigen::Vector2d arm_vector = body_position_[LShoulder] - body_position_[LWrist];
    ROS_INFO_STREAM("L-Elbow : (" << elbow_vector.coeff(0) << ", " << elbow_vector.coeff(1) << ")");

    calc_result = calcJointAngle(shoulder_vector,elbow_vector, l_elbow);

    Eigen::Vector3d elbow_vector_3d, arm_vector_3d;
    elbow_vector_3d << elbow_vector , 0;
    arm_vector_3d << arm_vector, 0;
    Eigen::Vector3d elbow_direction = elbow_vector_3d.cross(arm_vector_3d);

    if(elbow_direction.coeff(2) < 0)
      l_elbow *= (-1);

    if(is_upside_left == true)
    {
      std::map<std::string, double>::iterator joints_it;

      joints_it = joint_angles_.find("l_sho_pitch");
      if (joints_it != joint_angles_.end())
        joint_angles_["l_sho_pitch"] = M_PI - joint_angles_["l_sho_pitch"];
      else
        joint_angles_["l_sho_pitch"] = M_PI;

      joints_it = joint_angles_.find("l_sho_roll");
      if (joints_it != joint_angles_.end())
        joint_angles_["l_sho_pitch"] = - joint_angles_["l_sho_pitch"];
    }

    //    if(calc_result == true)
    //      ROS_INFO_STREAM("Left_Elbow : " << (l_elbow * 180 / M_PI));

    checkMaxAngle(120 * M_PI / 180.0, l_elbow);
    joint_angles_["l_el"] = l_elbow;
  }

  // calc right arm

  if(body_position_[RShoulder] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[RElbow] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[RWrist] != Eigen::Vector2d(0.0, 0.0))
  {
    double r_elbow = 0.0;
    calc_result = false;

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "LShoulder : (" << body_position_[RShoulder].coeff(0) << ", " << body_position_[RShoulder].coeff(1)
                         << "), LElbow : (" << body_position_[RElbow].coeff(0) << ", " << body_position_[RElbow].coeff(1)
                         << "), LWrist : (" << body_position_[RWrist].coeff(0) << ", " << body_position_[RWrist].coeff(1) << ")");

    Eigen::Vector2d shoulder_vector = body_position_[RShoulder] - body_position_[RElbow];
    Eigen::Vector2d elbow_vector = body_position_[RElbow] - body_position_[RWrist];
    Eigen::Vector2d arm_vector = body_position_[RShoulder] - body_position_[RWrist];
    ROS_INFO_STREAM_COND(DEBUG_PRINT, "R-Elbow : (" << elbow_vector.coeff(0) << ", " << elbow_vector.coeff(1) << ")");

    calc_result = calcJointAngle(shoulder_vector, elbow_vector, r_elbow);

    //    if(calc_result == true)
    //      ROS_INFO_STREAM("Right_Elbow : " << (r_elbow * 180 / M_PI));

    Eigen::Vector3d elbow_vector_3d, arm_vector_3d;
    elbow_vector_3d << elbow_vector , 0;
    arm_vector_3d << arm_vector, 0;
    Eigen::Vector3d elbow_direction = elbow_vector_3d.cross(arm_vector_3d);

    if(elbow_direction.coeff(2) < 0)
      r_elbow *= (-1);

    checkMaxAngle(120 * M_PI / 180.0, r_elbow);
    joint_angles_["r_el"] = r_elbow;
  }

  // calc head : it will not move

  // calc lower body : it will not move

}

void MotionFollower::publishJointStates()
{  
  if(joint_angles_.size() == 0)
    return;

  sensor_msgs::JointState op3_joints;

  for (std::map<std::string, double>::iterator pub_joint_it = joint_angles_.begin(); pub_joint_it != joint_angles_.end(); ++pub_joint_it)
  {
    op3_joints.name.push_back(pub_joint_it->first);
    op3_joints.position.push_back(pub_joint_it->second);
    ROS_INFO_STREAM(pub_joint_it->first << " : " << (pub_joint_it->second * 180 / M_PI));
  }

  // publish jointstates
  if(op3_joints.name.size() != 0)
    op3_joints_pub_.publish(op3_joints);
}

bool MotionFollower::calcJointAngle(Eigen::Vector2d upper, Eigen::Vector2d lower, double& target_angle)
{
  // no data
  if(upper == -lower)
    return false;

  // calc
  double inter_value = upper.dot(lower) / (upper.norm() * lower.norm());
  target_angle = acos(inter_value);

  return true;
}


void MotionFollower::checkMaxAngle(const double maximun, double &target_angle)
{
  if(fabs(target_angle) > maximun)
    target_angle = target_angle > 0 ? maximun : - maximun;
}

}
