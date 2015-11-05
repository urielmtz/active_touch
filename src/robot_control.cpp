/*
* Copyright: (C) 2015 Sheffield Robotics
* Authors: Uriel Martinez
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include <ros/ros.h>
#include <math.h>
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>

#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "schunk_sdh/TactileSensor.h"
#include "sensor_msgs/JointState.h"
#include "actionlib_msgs/GoalStatusArray.h"


#define MAX_TACTILE_SENSORS 6
#define MAX_TACTILE_PARAMS  78
#define MAX_JOINTS          7
#define THRESHOLD_CONTACT   100
#define MAX_JOINT_PARAMS    3
#define MAX_EXPLORATION_STEPS   25

using namespace std;

class RobotControl
{
	ros::NodeHandle node;

	ros::Publisher publishHand;
	ros::Publisher publishConfig;
	ros::Publisher publishMode;
 	ros::Publisher publishArmOrientation;

    ros::Subscriber subcribeContactStatus;
    ros::Subscriber subcribeActiveControl;
    ros::Subscriber subcribeArmOrientationStatus;
    ros::Subscriber subscribeArmGoalStatus;
    
    bool contactStatus;
    bool armControlStatus;
    bool armGoalStatus;
    float x_pos;
    float y_pos;
    float z_pos;
    
public:
	RobotControl()
	{		
		ROS_INFO("RobotControl started successfully");
        
        initConnections();

        contactStatus = false;
        armControlStatus = false;
        armGoalStatus = false;
        x_pos = 0.0;
        y_pos = 0.0;
        z_pos = 0.0;
        
	}

	~RobotControl()
	{}

	void initConnections()
	{

        publishHand = node.advertise<std_msgs::Float32MultiArray>("/hand_position", 1000);
        publishConfig = node.advertise<std_msgs::String>("/sensory_file_name", 1000);
        publishMode = node.advertise<std_msgs::String>("/sensory_file_mode", 1000);
        publishArmOrientation = node.advertise<std_msgs::Float32MultiArray>("/arm_control", 1000);
		
        subcribeContactStatus = node.subscribe("/contact_status", 1000, &RobotControl::getContactStatusCallback, this);
        subcribeArmOrientationStatus = node.subscribe("/arm_control_feedback", 1000, &RobotControl::getArmControlFeedbackCallback, this);        

        subscribeArmGoalStatus = node.subscribe("/sdh_controller/follow_joint_trajectory/status", 1000, &RobotControl::getArmGoalStatusCallback, this);        
        ros::spinOnce();
	}
    
    void getContactStatusCallback(const std_msgs::String::ConstPtr& msg)
    {
        // contact status published by tactile_sensors module
        if( msg->data == "contact_detected" || msg->data == "goal_reached" )
            contactStatus = true;	// contact detected published by tactile_sensors module
        else
            contactStatus = false;	// contact not detected
    }
    
    void goHome()
    {
        std_msgs::Float32MultiArray msg;
        //float totalOpen[7] = {1.047, -0.785, 1.047, -0.785, 1.047, -0.785, 1.047};    // sphere
        float totalOpen[MAX_JOINTS] = {1.047, -1.5, 1.047, -1.5, 1.047, -1.5, 1.047};    // sphere
        
        msg.data.clear();
        
        for(int i=0;i<MAX_JOINTS;i++)
        {
            msg.data.push_back(totalOpen[i]);
        }
        
        publishHand.publish(msg);
        ros::spinOnce();
        boost::this_thread::sleep( boost::posix_time::milliseconds(3000) );
    }
    
    void goTouch()
    {
        std_msgs::Float32MultiArray msg;
        float touchShape[MAX_JOINTS] = {1.047, -0.262, 1.047, -0.262, 1.047, -0.262, 1.047};
        
        msg.data.clear();
        
        for(int i=0;i<MAX_JOINTS;i++)
        {
            msg.data.push_back(touchShape[i]);
        }
        
        publishHand.publish(msg);
        ros::spinOnce();
        boost::this_thread::sleep( boost::posix_time::milliseconds(3000) );
    }
        
    void doDataCollection(std::string file)
    {
        std_msgs::String fileName;
        std_msgs::String fileMode;
        stringstream strCounter;


        int counter = 0;

        do
        {            
            strCounter << counter;
            fileName.data = file + "-" + strCounter.str();
            fileMode.data = "write";
            std::cout << "NAME: " << fileName.data << ", MODE: " << fileMode.data << std::endl;

            publishConfig.publish(fileName);
            ros::spinOnce();
            publishMode.publish(fileMode);
            ros::spinOnce();
            
            boost::this_thread::sleep( boost::posix_time::milliseconds(2000) );            

            goTouch();
            ros::spinOnce();

            // save tactile and proprioceptive data in text files
            do 
            {
                std::cout << "GOAL STATUS: " << armGoalStatus << std::endl;
                ros::spinOnce();
            }while( !armGoalStatus ); // do while force contact is not detected
            //}while( !contactStatus ); // do while tactile contact is not detected
            
            std::cout << "CONTACT DETECTED" << std::endl;

            boost::this_thread::sleep( boost::posix_time::milliseconds(1000) );

            fileMode.data = "none";
            publishMode.publish(fileMode);            
            ros::spinOnce();
            
            goHome();            

            counter++;            

            strCounter.str("");
            fileName.data = "";

            if( counter < MAX_EXPLORATION_STEPS )
            {
                doRotate(5.0);
            }
            
        }while( counter < MAX_EXPLORATION_STEPS );

        doRotate((counter-1)*(-5.0));
    }

    void doRotate(float yawValue)
    {
        std_msgs::Float32MultiArray msg;
        float tempYaw = yawValue;

        float yaw = (2*M_PI*tempYaw)/360.0;
        
        msg.data.resize(1);

        armControlStatus = false;
        msg.data[0] = yaw;
        publishArmOrientation.publish(msg);            
        ros::spinOnce();
        std::cout << "WAITING FOR COMPLATION: status: " << armControlStatus << std::endl;
                        
        while( !armControlStatus )
        {
            ros::spinOnce();
        }

        std::cout << "ROTATION COMPLETED: status: " << armControlStatus << std::endl;
    }
    
    void doTraining()
    {
        std_msgs::Float32MultiArray msg;
        float tempYaw = 10.0;

        float yaw = (2*M_PI*tempYaw)/360.0;
        
        msg.data.resize(1);

        for( int i = 0; i < 5; i++ )
        {
            armControlStatus = false;
            msg.data[0] = yaw;
            std::cout << "SENDING: " << msg.data[0] << std::endl;
            publishArmOrientation.publish(msg);            
            ros::spinOnce();
            std::cout << "WAITING FOR COMPLATION: status: " << armControlStatus << std::endl;
                        
            while( !armControlStatus )
            {
                ros::spinOnce();
            }

            std::cout << "ROTATION COMPLETED: status: " << armControlStatus << std::endl;
            
            yaw = (2*M_PI*tempYaw)/360.0;
        }
    }
    
    void doActiveExploration()
    {
    }
    
    void getArmControlFeedbackCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        armControlStatus = msg->data;
    }
    
    void getArmGoalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
    {
        // contact status published by sdh_controller module
        if( msg->status_list.size() > 1 )
            armGoalStatus = true;
        else
            armGoalStatus = false;
    }    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RobotControl");
    RobotControl myRobotControl;  

    std::string command = "";
    bool task_status = true;    

    do
    {
        std::cout << "Command: ";
        command = "";
        std::cin >> command;

        if( command == "home" )
          myRobotControl.goHome();  // open hand
        else if( command == "touch" )
          myRobotControl.goTouch(); // close hand
        else if( command == "collect" )
          myRobotControl.doDataCollection("dataCollection-test-03112015"); // data collection, control of fingers and wrist: dataCollection-type-mode-date
        else if( command == "training" )
          myRobotControl.doTraining(); // method for training.
        else if( command == "explore" )
          myRobotControl.doActiveExploration(); // method for active exploration.
        else if( command == "exit" )
          task_status = false;
        else
          std::cout << "Command not recognised!" << std::endl;          
        
        ros::spinOnce();
    }while( task_status );

    return 0;
}
