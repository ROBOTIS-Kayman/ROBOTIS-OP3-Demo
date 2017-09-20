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

#ifndef OP3_MOTION_FOLLOWER_H
#define OP3_MOTION_FOLLOWER_H

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <cmath>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include "openpose_ros_msgs/Persons.h"
#include "robotis_math/robotis_linear_algebra.h"

namespace robotis_op
{

class MotionFollower
{
 public:

  MotionFollower();
  ~MotionFollower();

  // const

  // enum

  // method

  // variable

 protected:

  // const
  const std::string MODULE_NAME;
  const bool DEBUG_PRINT;

  // enum
  enum BodyParts
  {
    Nose = 0,
    Neck = 1,
    RShoulder = 2,
    RElbow = 3,
    RWrist = 4,
    LShoulder = 5,
    LElbow = 6,
    LWrist = 7,
    RHip = 8,
    RKnee = 9,
    RAnkle = 10,
    LHip = 11,
    LKnee = 12,
    LAnkle = 13,
    REye = 14,
    LEye = 15,
    REar = 16,
    LEar = 17
  };

  // method
  void humanPoseCallback(const openpose_ros_msgs::Persons::ConstPtr& msg);
  void buttonCallback(const std_msgs::String::ConstPtr& msg);

  void checkTorque();
  void goInitPose();
  void setModule(const std::__cxx11::string &module_name);
  void parseInit();
  bool getShoulderLength(const openpose_ros_msgs::PersonDetection &person, double &length);
  void calcJointStates(const openpose_ros_msgs::PersonDetection &person_to_follow);
  void publishJointStates();

  bool calcJointAngle(Eigen::Vector2d upper, Eigen::Vector2d lower, double &target_angle);

  void checkMaxAngle(const double maximun, double &target_angle);

  //ros node handle
  ros::NodeHandle nh_;

  //image publisher/subscriber
  ros::Subscriber human_pose_sub_;
  ros::Subscriber button_sub_;
  ros::Publisher op3_joints_pub_;
  ros::Publisher dxl_torque_pub_;
  ros::Publisher set_module_pub_;

  // variable
  openpose_ros_msgs::PersonDetection person_to_follow_;
  std::map <int, Eigen::Vector2d> body_position_;
  std::map<std::string, double> joint_angles_;

  Eigen::Vector3d l_shoulder_3d_, r_shoulder_3d_;

  bool is_ready_;
};
}

#endif // OP3_MOTION_FOLLOWER_H
