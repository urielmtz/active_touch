/*
* Copyright: (C) 2015 Sheffield Robotics
* Authors: Uriel Martinez
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include "schunk_sdh/TactileSensor.h"
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <fstream>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>

#define MAX_TACTILE_SENSORS 6
#define MAX_TACTILE_PARAMS  78
#define MAX_JOINTS          7
#define THRESHOLD_CONTACT   30000
#define MAX_JOINT_PARAMS    3

using namespace std;


class SensoryFeedback
{
    private:

        ros::NodeHandle node;
        ros::Subscriber subJointsData;
        ros::Subscriber subTactileData;
        ros::Subscriber subFileName;
        ros::Subscriber subFileMode;
        
        ros::Publisher pubContactStatus;

        std::vector<std::vector<float> > tactileData;
        std::vector<std::vector<float> > jointData;

        bool writeStatus;
        std_msgs::String contactStatus;
        std::string tactileFileName;
        std::string jointsFileName;
        ofstream tactileDataFile;
        ofstream jointsDataFile;

    public:
        SensoryFeedback()
        {
            initConnections();
            initVariables();        

            writeStatus = false;
            contactStatus.data = "empty";
            tactileFileName = "empty.txt";
            jointsFileName = "empty.txt";
        }
        
        ~SensoryFeedback()
        {
        }

    	void initConnections()
    	{
            subTactileData = node.subscribe("/dsa_controller/tactile_data", 1000, &SensoryFeedback::getTactileDataCallback, this);
            subJointsData = node.subscribe("/joint_states", 1000, &SensoryFeedback::getJointsDataCallback, this);
            subFileName = node.subscribe("/sensory_file_name", 1000, &SensoryFeedback::getFileNameCallback, this);
            subFileMode = node.subscribe("/sensory_file_mode", 1000, &SensoryFeedback::getFileModeCallback, this);
            
            pubContactStatus = node.advertise<std_msgs::String>("/contact_status", 1000);
    	}

        void initVariables()
        {
            tactileData.resize(MAX_TACTILE_SENSORS);
            for( int i = 0; i < MAX_TACTILE_SENSORS; i++ )
                tactileData.at(i).resize(MAX_TACTILE_PARAMS);

            jointData.resize(MAX_JOINTS);
            for( int i = 0; i < MAX_JOINTS; i++ )
                jointData.at(i).resize(MAX_JOINT_PARAMS);            
        }

        void getTactileDataCallback(const schunk_sdh::TactileSensor::ConstPtr& msg)
        {
            float sum = 0.0;
            for( int i = 0; i < MAX_TACTILE_SENSORS; i++ )
            {
                //int nvalues = (msg->tactile_matrix[i].cells_x * msg->tactile_matrix[i].cells_y);
            
                //for(int j = 0; j < nvalues; j++ )
                for( int j = 0; j < MAX_TACTILE_PARAMS; j++ )
                {
                    tactileData[i][j] = msg->tactile_matrix[i].tactile_array[j];
                    sum = sum + tactileData[i][j];
                    
                    if( writeStatus )
                    {
                        tactileDataFile << tactileData[i][j] << " ";
                    }
                    
                    if( sum > THRESHOLD_CONTACT )
                        contactStatus.data = "contact_detected";
                    else
                        contactStatus.data = "empty";
                    
                    pubContactStatus.publish(contactStatus);
                }

                if( writeStatus )
                {
                    tactileDataFile << std::endl;
                }
            }
        }

        void getJointsDataCallback(const sensor_msgs::JointState::ConstPtr& msg)
        {        
            if( msg->name[0] == "sdh_finger_21_joint" )
            {
                // Not used for this process
            }
            else
            {
                for(int i = 0; i < MAX_JOINTS; i++)
                {
                    jointData[i][0] = msg->position[i];
                    jointData[i][1] = msg->velocity[i];
                    jointData[i][2] = msg->effort[i];
                    
                    if( writeStatus )
                    {
                        jointsDataFile << jointData[i][0] << " ";
                    }
                }

                if( writeStatus )
                {
                    jointsDataFile << std::endl;
                }
            }    
        }

        void getFileNameCallback(const std_msgs::String::ConstPtr& msg)
        {
            tactileFileName = msg->data + "-tactile.txt";          
            jointsFileName = msg->data + "-joints.txt";
        }

        void getFileModeCallback(const std_msgs::String::ConstPtr& msg)
        {
            std::string temp = msg->data;
            
            if( "write" == temp )
            {
                if( !tactileDataFile.is_open() && !jointsDataFile.is_open() )
                {
                    std::cout << "\n\n ===== Creating files: " << tactileFileName << " and " << jointsFileName;
                    tactileDataFile.open(tactileFileName.c_str());
                    jointsDataFile.open(jointsFileName.c_str());
                    writeStatus = true;
                }
            }
            else
            {
                if( tactileDataFile.is_open() && jointsDataFile.is_open() )
                {
                    std::cout << "\n\n ===== Closing files: " << tactileFileName << " and " << jointsFileName;
                    tactileDataFile.close();
                    jointsDataFile.close();
                    writeStatus = false;
                }                    
            }
            
            std::cout << "\n\n ===== mode: " << msg->data;
        }
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "SensoryFeedback");

    SensoryFeedback objSensoryFeedback;

    ros::spin();

    return 0;
}
