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

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

private:
	
    bool readParameters();
    // void minRangeCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    // void pointNumCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
	void pillarCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
	/* 用“指针的引用”或“引用”可以只传地址，不能用“::Ptr” 因为可能会不小心对传感器数据进行修改*/
    float pillarOrientation(const sensor_msgs::LaserScan::ConstPtr &msg);
	float pillarDistance(const sensor_msgs::LaserScan::ConstPtr &msg);
	bool start_control(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    ros::NodeHandle nodeHandle_;
    // ros::Subscriber scan_subscriber_;
	// ros::Subscriber point_subscriber_;
	ros::Subscriber pillar_subscriber_;
	ros::Publisher pillar_publisher_;
	ros::Publisher marker_publisher_;

    std::string scan_subscriber_topic_;
	std::string point_subscriber_topic_;
	std::string service_name_;

	ros::ServiceServer service_;
	// tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener;

    int queue_size_;
    float p_controller_;
	geometry_msgs::Twist cmd_vel_;
	// geometry_msgs::PointStamped marker_point_;
	visualization_msgs::Marker marker_;
	
	bool start = 1;
};

} /* namespace */


