
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <cmath>

// 假设这个头文件提供了所需的辅助函数如 init_publisher_subscriber(), wait4connect(), wait4start(), takeoff(), land()
#include "gnc_functions.hpp"

sensor_msgs::NavSatFix current_position;

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    current_position = *msg;
    ROS_INFO("Current GPS: [%f, %f, %f]", msg->latitude, msg->longitude, msg->altitude);
}

void set_global_position_target(double latitude, double longitude, double altitude, ros::NodeHandle& nh) {
    ros::Publisher global_position_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 10);
    
    mavros_msgs::GlobalPositionTarget target;
    target.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
    target.latitude = latitude;
    target.longitude = longitude;
    target.altitude = 585.2;
   target.yaw = 90;
    global_position_pub.publish(target);
}

double calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000; // 地球半径，单位为米
    double latRad1 = lat1 * M_PI / 180;
    double latRad2 = lat2 * M_PI / 180;
    double deltaLatRad = (lat2 - lat1) * M_PI / 180;
    double deltaLonRad = (lon2 - lon1) * M_PI / 180;

    double a = sin(deltaLatRad / 2) * sin(deltaLatRad / 2) +
               cos(latRad1) * cos(latRad2) *
               sin(deltaLonRad / 2) * sin(deltaLonRad / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    double distance = R * c; // 最终距离
    return distance;
}
struct TargetPoint {
    double latitude;
    double longitude;
    double altitude;
};

std::vector<TargetPoint> waypoints = {
    {-35.363261, 149.16523, 603.7887287},       // 初始位置
    {-35.36325199, 149.16523, 603.7887287},     //North 1
    {-35.36325199, 149.16524100, 603.7887287},  // est 1
    {-35.36327001, 149.16524100, 603.7887287},  //south 2
    {-35.36327001, 149.16521899, 603.7887287},  // west 2
    {-35.36325199, 149.16521899, 603.7887287},  // north 2  
    
    {-35.363261, 149.16523, 603.7887287},
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle nh;
    initialize_local_frame();

    init_publisher_subscriber(nh);
    
    ros::Subscriber position_sub = nh.subscribe("/mavros/global_position/global", 10, globalPositionCallback);
    
    wait4connect();
    wait4start();

    takeoff(1.2);

    ros::Rate rate(2.0); // 2 Hz
    size_t current_waypoint_index = 0; // 当前目标点索引

    while(ros::ok() && current_waypoint_index < waypoints.size()) {
        auto& target = waypoints[current_waypoint_index];
        
        set_global_position_target(target.latitude, target.longitude, target.altitude, nh);
        
        double current_distance = calculate_distance(current_position.latitude, current_position.longitude, target.latitude, target.longitude);
        ROS_INFO("Current distance to waypoint %lu: %f meters", current_waypoint_index, current_distance);
       
        // 检查是否到达目标点（例如，距离小于10米）
        if (current_distance < 0.5) {
            ROS_INFO("Arrived at waypoint %lu.", current_waypoint_index);
            current_waypoint_index++; // 移动到下一个目标点

            if (current_waypoint_index >= waypoints.size()) {
                ROS_INFO("All waypoints reached. Preparing to land.");
                land();
                break;
            }
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}