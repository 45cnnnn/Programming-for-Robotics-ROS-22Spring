#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
    if(!readParameters())
    {
        ROS_ERROR("Cannot find parameter");
        ros::requestShutdown();
    }
    // scan_subscriber_ = nodeHandle_.subscribe(scan_subscriber_topic_, queue_size_, &SmbHighlevelController::minRangeCallback, this);
    // point_subscriber_ = nodeHandle_.subscribe(point_subscriber_topic_, queue_size_, &SmbHighlevelController::pointNumCallback, this);
    pillar_subscriber_ = nodeHandle_.subscribe(scan_subscriber_topic_, queue_size_, &SmbHighlevelController::pillarCallback, this);
    pillar_publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", queue_size_);
    marker_publisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
}

bool SmbHighlevelController::readParameters()
{
    if(!(nodeHandle_.getParam("/smb_highlevel_controller/scan_subscriber_topic", scan_subscriber_topic_ ))) return false;
    if(!(nodeHandle_.getParam("/smb_highlevel_controller/point_subscriber_topic", point_subscriber_topic_ ))) return false;
    if(!(nodeHandle_.getParam("/smb_highlevel_controller/queue_size", queue_size_ ))) return false;
    if(!(nodeHandle_.getParam("/smb_highlevel_controller/p_controller", p_controller_ ))) return false;
    return true;
}

/* Exercise 2: Calculate the smallest distance measurement from the vector ranges in the message of the laser scanner*/
// void SmbHighlevelController::minRangeCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
// {
//     float min = msg->range_max;
//     int num;
//     int max_num = msg->ranges.size();
//     for(int i = 0; i < max_num; i++)
//     {
//         float dist = msg->ranges.at(i);
//         if (dist < min)
//         {
//             min = dist;
//         }
//         num++;
//     }
//     ROS_INFO( "smallest distance is : %f", min);
//     // ROS_INFO_STREAM( "smallest distance is " << 1);

// }

/* Exercise 3: Calculate how many points the PointCloud has*/
// void SmbHighlevelController::pointNumCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
// {
//     // ROS_INFO( "3D point cloud has : %d points", msg.width);
//     // ROS_INFO( "3D point cloud has : %d points", msg.height);
//     // ROS_INFO( "3D point cloud has : %d points", msg.data.size());
//     ROS_INFO_STREAM( "3D point cloud has : " << msg->data.size());

// }

/* Exercise 3: Extract the distance of the pillar from the laser scan wrt the robot*/
float SmbHighlevelController::pillarDistance(const sensor_msgs::LaserScan::ConstPtr &msg){
    // float min = *std::min_element(msg.begin(), msg.end());
    float min = msg->range_max;
    int num;
    int max_num = msg->ranges.size();
    for(int i = 0; i < max_num; i++)
    {
        float dist = msg->ranges.at(i);
        if (dist < min)
        {
            min = dist;
        }
        num++;
    }
    return min;
}

/* Exercise 3: Extract the Orientation of the pillar from the laser scan wrt the robot*/
float SmbHighlevelController::pillarOrientation(const sensor_msgs::LaserScan::ConstPtr &msg){
    sensor_msgs::LaserScan::_ranges_type data;
    data = msg->ranges;
    int min_index;
    min_index = std::min_element(data.begin(),data.end()) - data.begin();
    float angle;
    angle = msg->angle_min + msg->angle_increment * min_index;
    return angle;
}

void SmbHighlevelController::pillarCallback(const sensor_msgs::LaserScan::ConstPtr &msg){
    float distance = pillarDistance(msg);
    float orientation = pillarOrientation(msg);
    // float position_x;
    // float position_y;
    ROS_INFO_STREAM("Distance from pillar is " << distance << "m");
    ROS_INFO_STREAM("Orientation of pillar is " << orientation << "degrees");
    
    cmd_vel_.linear.x = 1;
    cmd_vel_.angular.z = p_controller_*orientation;
    
    
    pillar_publisher_.publish(cmd_vel_);

    // while(nodeHandle_.ok()){
    //     geometry_msgs::TransformStamped transformStamped;
    //     try{
    //         transformStamped = tfBuffer.lookupTransform("rslidar", "odom", ros::Time(0));
    //     }
    //     catch(tf2::TransformException &exception){
    //         ROS_WARN("%s",exception.what());
    //         ros::Duration(1.0).sleep();
    //     }
    // }

    marker_.header.frame_id = "rslidar";
    // marker_.header.frame_id = "odom";
    marker_.header.stamp = ros::Time();
    marker_.ns = "my_namespace";
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::ARROW;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.x = distance*std::cos(orientation);
    marker_.pose.position.y = distance*std::sin(orientation);
    marker_.pose.position.z = 1;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 1;
    marker_.scale.y = 0.1;
    marker_.scale.z = 0.1;
    marker_.color.a = 1.0;
    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 0.0;

    marker_publisher_.publish(marker_);
}

SmbHighlevelController::~SmbHighlevelController()
{
}

} /* namespace */
