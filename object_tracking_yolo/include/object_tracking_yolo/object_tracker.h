/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman Jung */

#ifndef OBJECT_TRACKING_H_
#define OBJECT_TRACKING_H_

#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <yaml-cpp/yaml.h>

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

namespace robotis_op
{

// head tracking for looking the object
class ObjectTracker
{
public:
  // const

  // enum
  enum TrackingStatus
  {
    NotFound = -1,
    Waiting = 0,
    Found = 1,

  };
  enum TrackingCommand
  {
    NoCommand = -1,
    StopTracking = 0,
    StartTracking = 1,
    SpeakObject = 2,
  };


  // method
  ObjectTracker();
  ~ObjectTracker();

  int processTracking();

  void startTracking();
  void stopTracking();

  void setBeingScanning(bool use_scan);

  double getPanOfObject()
  {
    // left (+) ~ right (-)
    return object_current_pan_;
  }
  double getTiltOfObject()
  {
    // top (+) ~ bottom (-)
    return object_current_tilt_;
  }
  double getObjectSize()
  {
    return object_current_size_;
  }

  // variable

protected:
  const double FOV_WIDTH;
  const double FOV_HEIGHT;
  const int NOT_FOUND_THRESHOLD;
  const int WAITING_THRESHOLD;
  const bool DEBUG_PRINT;
  const int INIT_POSE_INDEX;
  std::string object_start_command_;
  std::string object_stop_command_;
  std::string object_speak_command_;
  std::string object_target_;

  void objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
  void buttonHandlerCallback(const std_msgs::String::ConstPtr &msg);
  void publishHeadJoint(double pan, double tilt);

  int getCommandFromObject(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
  void getTargetFromMsg(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);

  void getConfig(const std::string &config_path);
  void setModule(const std::string &module_name);
  void setLED(const int led_value);

  void readyToDemo();
  void playMotion(int motion_index);

  // test code
  void getROSCommand(const std_msgs::String::ConstPtr &msg);

  //ros node handle
  ros::NodeHandle nh_;

  //image publisher/subscriber
  ros::Publisher set_module_pub_;
  ros::Publisher module_control_pub_;
  ros::Publisher head_joint_pub_;
  ros::Publisher led_pub_;
  ros::Publisher motion_index_pub_;
  ros::Publisher head_scan_pub_;

  ros::Subscriber object_sub_;
  ros::Subscriber buttuon_sub_;
  ros::Subscriber ball_tracking_command_sub_;

  ros::Subscriber test_sub_;

  // (x, y) is the center position of the ball in image coordinates
  // z is the ball radius
  geometry_msgs::Point object_position_;
  geometry_msgs::Point prev_position_;

  int tracking_status_;
  bool use_head_scan_;
  int count_not_found_;
  bool on_tracking_;
  double object_current_pan_, object_current_tilt_;
  double object_current_size_;
  double x_error_sum_, y_error_sum_;
  ros::Time prev_time_;
  double p_gain_, d_gain_, i_gain_;

};
}

#endif /* OBJECT_TRACKING_H_ */
