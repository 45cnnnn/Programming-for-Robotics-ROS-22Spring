#include <ros/ros.h>
#include "smb_highlevel_controller/SmbObstacleDetection.hpp"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "smb_highlevel_controller");
    ros::NodeHandle nodeHandle("~");

    smb_highlevel_controller::SmbObstacleDetection smbObstacleDetection(nodeHandle);

    ros::spin();
    return 0;
}