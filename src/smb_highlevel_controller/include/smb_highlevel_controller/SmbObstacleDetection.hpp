#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>

namespace smb_highlevel_controller {

class SmbObstacleDetection{
public:
	/*!
	 * Constructor.
	 */
	SmbObstacleDetection(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~SmbObstacleDetection();

private:
    ros::NodeHandle nodeHandle_;
    ros::ServiceClient obs_client_;
    ros::Subscriber subscriber_;

    bool readParamters();
    bool distanceMeasure(const sensor_msgs::LaserScan::ConstPtr& msg);
    void obsCallback(const sensor_msgs::LaserScan::ConstPtr& msg);


    std::string scan_subscriber_topic_;
    int queue_size_;
    std::string service_name_;
    std::string service_name_obs_;
    };

}
