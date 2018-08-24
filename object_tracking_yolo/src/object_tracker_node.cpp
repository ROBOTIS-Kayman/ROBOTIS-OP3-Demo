#include <ros/ros.h>
#include "object_tracking_yolo/object_tracker.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_tracker_node");
  ros::NodeHandle nh;

  robotis_op::ObjectTracker *op3_object_tracker = new robotis_op::ObjectTracker();

  ROS_INFO("Start object tracking demo!");

  //set node loop rate
  ros::Rate loop_rate(25);

  //node loop
  while (ros::ok())
  {
    op3_object_tracker->processTracking();

    //relax to fit output rate
    loop_rate.sleep();
    ros::spinOnce();
  }

}
