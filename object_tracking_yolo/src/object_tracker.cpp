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

#include "object_tracking_yolo/object_tracker.h"

namespace robotis_op
{

ObjectTracker::ObjectTracker()
  : nh_(ros::this_node::getName()),
    FOV_WIDTH(35.2 * M_PI / 180),
    FOV_HEIGHT(21.6 * M_PI / 180),
    NOT_FOUND_THRESHOLD(50),
    WAITING_THRESHOLD(5),
    use_head_scan_(true),
    count_not_found_(0),
    on_tracking_(false),
    object_current_pan_(0),
    object_current_tilt_(0),
    x_error_sum_(0),
    y_error_sum_(0),
    object_current_size_(0),
    tracking_status_(NotFound),
    COMMAND_START("sports ball"),
    COMMAND_STOP("teddy bear"),
    COMMAND_SPEAK("banana"),
    TARGET_OBJECT("cell phone"),
    DEBUG_PRINT(true)
{
  ros::NodeHandle param_nh("~");
  p_gain_ = param_nh.param("p_gain", 0.4);
  i_gain_ = param_nh.param("i_gain", 0.0);
  d_gain_ = param_nh.param("d_gain", 0.0);

  // get config
  std::string default_config_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/config/config.yaml";
  getConfig(default_config_path);

  ROS_INFO_STREAM("Object tracking Gain : " << p_gain_ << ", " << i_gain_ << ", " << d_gain_);

  head_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states_offset", 0);
//  head_scan_pub_ = nh_.advertise<std_msgs::String>("/robotis/head_control/scan_command", 0);
  //  error_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/ball_tracker/errors", 0);

  object_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &ObjectTracker::objectCallback, this);
//  ball_tracking_command_sub_ = nh_.subscribe("/ball_tracker/command", 1, &ObjectTracker::ballTrackerCommandCallback, this);

  test_sub_ = nh_.subscribe("/ros_command", 1, &ObjectTracker::getROSCommand, this);
}

ObjectTracker::~ObjectTracker()
{

}

//void ObjectTracker::ballPositionCallback(const op3_ball_detector::CircleSetStamped::ConstPtr &msg)
//{
//  for (int idx = 0; idx < msg->circles.size(); idx++)
//  {
//    if (ball_position_.z >= msg->circles[idx].z)
//      continue;

//    ball_position_ = msg->circles[idx];
//  }
//}

//void ObjectTracker::ballTrackerCommandCallback(const std_msgs::String::ConstPtr &msg)
//{
//  if (msg->data == "start")
//  {
//    startTracking();
//  }
//  else if (msg->data == "stop")
//  {
//    stopTracking();
//  }
//  else if (msg->data == "toggle_start")
//  {
//    if (on_tracking_ == false)
//      startTracking();
//    else
//      stopTracking();
//  }
//}

void ObjectTracker::startTracking()
{
  on_tracking_ = true;
  ROS_INFO_COND(DEBUG_PRINT, "Start Object tracking");
}

void ObjectTracker::stopTracking()
{
  on_tracking_ = false;
  ROS_INFO_COND(DEBUG_PRINT, "Stop Object tracking");

  object_current_pan_ = 0;
  object_current_tilt_ = 0;
  x_error_sum_ = 0;
  y_error_sum_ = 0;
}

void ObjectTracker::setBeingScanning(bool use_scan)
{
  use_head_scan_ = use_scan;
}

int ObjectTracker::processTracking()
{
  int tracking_status = Found;

  if (on_tracking_ == false)
  {
//    object_position_.z = 0;
//    count_not_found_ = 0;
    return NotFound;
  }

  // check object position
  if (object_position_.z <= 0)
  {
    count_not_found_++;

    if (count_not_found_ < WAITING_THRESHOLD)
    {
      if(tracking_status_ == Found || tracking_status_ == Waiting)
        tracking_status = Waiting;
      else
        tracking_status = NotFound;
    }
    else if (count_not_found_ > NOT_FOUND_THRESHOLD)
    {
//      scanBall();
      count_not_found_ = 0;
      tracking_status = NotFound;
    }
    else
    {
      tracking_status = NotFound;
    }
  }
  else
  {
    count_not_found_ = 0;
  }

  // if ball is found
  // convert ball position to desired angle(rad) of head
  // ball_position : top-left is (-1, -1), bottom-right is (+1, +1)
  // offset_rad : top-left(+, +), bottom-right(-, -)
  double x_error = 0.0, y_error = 0.0, ball_size = 0.0;

  switch (tracking_status)
  {
  case NotFound:
    tracking_status_ = tracking_status;
    object_current_pan_ = 0;
    object_current_tilt_ = 0;
    x_error_sum_ = 0;
    y_error_sum_ = 0;
    return tracking_status;

  case Waiting:
    tracking_status_ = tracking_status;
    return tracking_status;

  case Found:
    x_error = -atan(object_position_.x * tan(FOV_WIDTH));
    y_error = -atan(object_position_.y * tan(FOV_HEIGHT));
    ball_size = object_position_.z;
    break;

  default:
    break;
  }

  ROS_INFO_STREAM_COND(DEBUG_PRINT, "--------------------------------------------------------------");
  ROS_INFO_STREAM_COND(DEBUG_PRINT, "Ball position : " << object_position_.x << " | " << object_position_.y);
  ROS_INFO_STREAM_COND(DEBUG_PRINT, "Target angle : " << (x_error * 180 / M_PI) << " | " << (y_error * 180 / M_PI));

  ros::Time curr_time = ros::Time::now();
  ros::Duration dur = curr_time - prev_time_;
  double delta_time = dur.nsec * 0.000000001 + dur.sec;
  prev_time_ = curr_time;

  double x_error_diff = (x_error - object_current_pan_) / delta_time;
  double y_error_diff = (y_error - object_current_tilt_) / delta_time;
  x_error_sum_ += x_error;
  y_error_sum_ += y_error;
  double x_error_target = x_error * p_gain_ + x_error_diff * d_gain_ + x_error_sum_ * i_gain_;
  double y_error_target = y_error * p_gain_ + y_error_diff * d_gain_ + y_error_sum_ * i_gain_;

  //  std_msgs::Float64MultiArray x_error_msg;
  //  x_error_msg.data.push_back(x_error);
  //  x_error_msg.data.push_back(x_error_diff);
  //  x_error_msg.data.push_back(x_error_sum_);
  //  x_error_msg.data.push_back(x_error * p_gain_);
  //  x_error_msg.data.push_back(x_error_diff * d_gain_);
  //  x_error_msg.data.push_back(x_error_sum_ * i_gain_);
  //  x_error_msg.data.push_back(x_error_target);
  //  error_pub_.publish(x_error_msg);

  ROS_INFO_STREAM_COND(DEBUG_PRINT, "------------------------  " << tracking_status << "  --------------------------------------");
  ROS_INFO_STREAM_COND(DEBUG_PRINT, "error         : " << (x_error * 180 / M_PI) << " | " << (y_error * 180 / M_PI));
  ROS_INFO_STREAM_COND(
        DEBUG_PRINT,
        "error_diff    : " << (x_error_diff * 180 / M_PI) << " | " << (y_error_diff * 180 / M_PI) << " | " << delta_time);
  ROS_INFO_STREAM_COND(
        DEBUG_PRINT,
        "error_sum    : " << (x_error_sum_ * 180 / M_PI) << " | " << (y_error_sum_ * 180 / M_PI));
  ROS_INFO_STREAM_COND(
        DEBUG_PRINT,
        "error_target  : " << (x_error_target * 180 / M_PI) << " | " << (y_error_target * 180 / M_PI) << " | P : " << p_gain_ << " | D : " << d_gain_ << " | time : " << delta_time);

  // move head joint
  publishHeadJoint(x_error_target, y_error_target);

  // args for following ball
  object_current_pan_ = x_error;
  object_current_tilt_ = y_error;
  object_current_size_ = ball_size;

  object_position_.z = 0;

  tracking_status_ = tracking_status;
  return tracking_status;
}

void ObjectTracker::publishHeadJoint(double pan, double tilt)
{
  double min_angle = 1 * M_PI / 180;
  if (fabs(pan) < min_angle && fabs(tilt) < min_angle)
    return;

  sensor_msgs::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");

  head_angle_msg.position.push_back(pan);
  head_angle_msg.position.push_back(tilt);

  head_joint_pub_.publish(head_angle_msg);
}

//void ObjectTracker::scanBall()
//{
//  if (use_head_scan_ == false)
//    return;

//  // check head control module enabled
//  // ...

//  // send message to head control module
//  std_msgs::String scan_msg;
//  scan_msg.data = "scan";

//  head_scan_pub_.publish(scan_msg);
//}

void ObjectTracker::objectCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
  ROS_WARN_STREAM_COND(DEBUG_PRINT, "Object Callback : " << msg->bounding_boxes.size());
  // Header : header
  // Header : image_header
  // BoundingBox[] : bounding_boxs

  // check command
  int command = getCommandFromObject(msg);

  ROS_INFO_STREAM_COND((command != NoCommand), "Command from object : " << command);

  // handle command
  switch(command)
  {
  default:
    break;

  case StartTracking:
    startTracking();
    break;

  case StopTracking:
    stopTracking();
    break;

  case SpeakObject:
    break;
  }

  //check the target
  getTargetFromMsg(msg);
}

int ObjectTracker::getCommandFromObject(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
  int command = NoCommand;

  for(int ix = 0; ix < msg->bounding_boxes.size(); ix++)
  {
    if(msg->bounding_boxes[ix].Class == COMMAND_START)
      command = StartTracking;
    else if(msg->bounding_boxes[ix].Class == COMMAND_STOP)
      command = StopTracking;
    else if(msg->bounding_boxes[ix].Class == COMMAND_SPEAK)
      command = SpeakObject;
  }

  return command;
}

void ObjectTracker::getTargetFromMsg(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
  for(int ix = 0; ix < msg->bounding_boxes.size(); ix++)
  {
    darknet_ros_msgs::BoundingBox& bounding_box = (darknet_ros_msgs::BoundingBox&) (msg->bounding_boxes[ix]);
    // darknet_ros_msgs::BoundingBox *bounding_box = static_cast<darknet_ros_msgs::BoundingBox*>(&(msg->bounding_boxes[ix]));
    if(bounding_box.Class == TARGET_OBJECT)
    {
      object_position_.x = (bounding_box.xmax + bounding_box.xmin) / 1280.0 - 1.0;
      object_position_.y = (bounding_box.ymax + bounding_box.ymin) / 720.0 - 1.0;

      double object_x = abs(bounding_box.xmax - bounding_box.xmin);
      double object_y = abs(bounding_box.ymax - bounding_box.ymin);
      object_position_.z = sqrt(object_x * object_x + object_y * object_y);

      ROS_ERROR_COND(DEBUG_PRINT, "Found Object");
      return;
    }
  }

  object_position_.z = -1;

}

// test
void ObjectTracker::getROSCommand(const std_msgs::String::ConstPtr &msg)
{
  std::string ros_command = "gnome-terminal -x sh -c '" + msg->data + "'";

  system(ros_command.c_str());
}

void ObjectTracker::getConfig(const std::string &config_path)
{
  if(config_path == "")
    return;

  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(config_path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // object : start, stop, target

  //
}


}

