/*
* Copyright: (C) 2015 Sheffield Robotics
* Authors: Uriel Martinez
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include "sensor_msgs/JointState.h"

#include <iostream>
#include <fstream>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

void getJointsSDHCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int nJoints = 7;
    ROS_INFO("PROPRIOCEPTIVE: SEQ [%d], FRAME ID [%s]", msg->header.seq, msg->header.frame_id.c_str());
    
    if( msg->name[0] == "sdh_finger_21_joint" )
    {
        printf("Joint[%d]\n",0);
        printf("Name: %s\n",msg->name[0].c_str());
        printf("Position, Velocity: %f, %f\n", msg->position[0], msg->velocity[0]);
    }
    else
    {
        for(int i = 0; i < nJoints; i++)
        {
            printf("Joint[%d]\n",i);
            printf("Name: %s\n",msg->name[i].c_str());
            printf("Position, Velocity, Effort: %f, %f, %f\n", msg->position[i], msg->velocity[i], msg->effort[i]);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "getJointsSDH");
    
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/joint_states", 1000, getJointsSDHCallback);

    ros::spin();

    return 0;
}
