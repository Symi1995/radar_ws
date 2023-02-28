#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <string>
#include <stdlib.h>


ros::Publisher pub_difference;
double radarDistance;

void setToDefault() {
    radarDistance = -1;
}

void differencePublisher()
{
    if(radarDistance != -1) {
        
        visualization_msgs::Marker marker;
        tf2::Quaternion myQuaternion;

        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "/radar_closest_point";
        marker.ns = "text";
        marker.id = 999;
        marker.type = 9;
        marker.action = 0;
        marker.pose.position.x = -5;
        marker.pose.position.y =  0;
        marker.pose.position.z = -3; //12 prezentáláshoz, különben -3

        myQuaternion.setRPY(0, 0, 0);
        marker.pose.orientation.w = myQuaternion.getW();
        marker.pose.orientation.x = myQuaternion.getX();
        marker.pose.orientation.y = myQuaternion.getY();
        marker.pose.orientation.z = myQuaternion.getZ();
        marker.scale.z = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.25);
        marker.frame_locked = false;
        
        std::stringstream ss;
        ss.precision(2);
        ss << std::fixed << "Radar closest point: " << radarDistance << " m";
        marker.text = ss.str();

        pub_difference.publish(marker);

        setToDefault();
    }
}


void radarDataCallback(const std_msgs::String::ConstPtr& msg)
{
    radarDistance = std::atof(msg->data.c_str());
    ROS_INFO("RADAR DISTANCE: %s", msg->data.c_str());
    differencePublisher();
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "radar_closest");
    ros::NodeHandle nh;

    setToDefault();

    pub_difference = nh.advertise<visualization_msgs::Marker>("radar_closest_point",0);
    ros::Subscriber radar_closest = nh.subscribe("radar_closest_distance", 100, radarDataCallback);
    
    ROS_INFO("radar_closest - ROS OK");
    
    ros::spin();

    return 0;
}