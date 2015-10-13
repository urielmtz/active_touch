#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include "schunk_sdh/TactileSensor.h"
#include "sensor_msgs/JointState.h"

void getTactileDataCallback(const schunk_sdh::TactileSensor::ConstPtr& msg)
{
    ROS_INFO("SEQ [%d], FRAME ID [%s]", msg->header.seq, msg->header.frame_id.c_str());

    for( int i = 0; i < 6; i++ )
    {
        ROS_INFO("MATRIX ID [%d], Cells X [%d], Cells Y [%d]", msg->tactile_matrix[i].matrix_id, msg->tactile_matrix[i].cells_x, msg->tactile_matrix[i].cells_y);
        ROS_INFO("TACTILE ARRAY");
    
        float sum = 0.0;
        int nvalues = (msg->tactile_matrix[i].cells_x * msg->tactile_matrix[i].cells_y);
	printf("n_values = %d", nvalues);
    
        for(int j = 0; j < nvalues; j++ )
        {
            sum = sum + msg->tactile_matrix[i].tactile_array[j];
            printf("%d ", msg->tactile_matrix[i].tactile_array[j]);
        }
        printf("\n");
        ROS_INFO("Sum of tactile values: %f", sum/nvalues);

        printf("\n");
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getTactileData");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/dsa_controller/tactile_data", 1000, getTactileDataCallback);

  ros::spin();

  return 0;
}
