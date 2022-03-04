#include <smb_highlevel_controller/SmbObstacleDetection.hpp>

namespace smb_highlevel_controller{
    
SmbObstacleDetection::SmbObstacleDetection(ros::NodeHandle& nodeHandle):
nodeHandle_(nodeHandle){
    if(!readParamters()){
        ROS_ERROR("Cannot find parameter");
        ros::requestShutdown();
    }
    subscriber_ = nodeHandle_.subscribe(scan_subscriber_topic_, queue_size_, &SmbObstacleDetection::obsCallback, this);
    obs_client_ = nodeHandle_.serviceClient<std_srvs::SetBool>(service_name_obs_);
}

bool SmbObstacleDetection::readParamters(){
    if(!(nodeHandle_.getParam("/smb_highlevel_controller/scan_subscriber_topic", scan_subscriber_topic_ ))) return false;
    if(!(nodeHandle_.getParam("/smb_highlevel_controller/queue_size", queue_size_ ))) return false;
    if(!(nodeHandle_.getParam("/smb_highlevel_controller/service_name_obs", service_name_obs_ ))) return false;

    return true;
}

bool SmbObstacleDetection::distanceMeasure(const sensor_msgs::LaserScan::ConstPtr& msg){
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
    if(min <= 1){return false;}
    else{return true;}

}

void SmbObstacleDetection::obsCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

    std_srvs::SetBool service;

    if(distanceMeasure(msg)){
        service.request.data = 1;
    }
    else{
        service.request.data = 0;
    }
    if(obs_client_.call(service)){
        if(service.request.data == 0){ROS_INFO("Too close to the obstacle");}
    }
    else{
        ROS_ERROR("Fail to call services");
    }
   
}
SmbObstacleDetection::~SmbObstacleDetection(){

}

}