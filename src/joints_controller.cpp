/*
* Copyright: (C) 2015 Sheffield Robotics
* Authors: Uriel Martinez
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include "sensor_msgs/JointState.h"


void getJointsSDHCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int nJoints = 7;
    ROS_INFO("PROPRIOCEPTIVE: SEQ [%d], FRAME ID [%s]", msg->header.seq, msg->header.frame_id.c_str());
    
    for(int i = 0; i < nJoints; i++)
    {
	ROS_INFO("Joint, Position, Velocity, Effort [%d]: %f, %f, %f ", i, msg->position[i], msg->velocity[i], msg->effort[i]);
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getJointsSDH");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/joint_states", 1000, getJointsSDHCallback);

  ros::Rate loop_rate(10);

  ros::spin();

  loop_rate.sleep();

  return 0;
}
