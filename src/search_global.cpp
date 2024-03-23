
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

void set_global_position_and_yaw(double latitude, double longitude, double altitude, ros::NodeHandle& nh) {
    ros::Publisher position_target_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 10);

    // 确保发布者有时间来连接到ROS主题
    ros::Rate rate(20);
    while (position_target_pub.getNumSubscribers() < 1) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::GlobalPositionTarget target;
    target.header.stamp = ros::Time::now();
    target.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;

    target.latitude = latitude;
    target.longitude = longitude;
    target.altitude = 585.2;
    target.yaw = M_PI / 2; //  est rad
    //target.yaw =  0;        // north 
     //target.yaw =  M_PI;     // south
   // target.yaw = 3*M_PI/2;   //  west

    
    position_target_pub.publish(target);
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

//searching are 1 waypoints
std::vector<TargetPoint> waypoints = {
{-35.36324170, 149.16507850, 603.7887287},
{-35.36324360, 149.16519760, 603.7887287},
{-35.36325070, 149.16519780, 603.7887287},
{-35.36324990, 149.16507850, 603.7887287},
{-35.36325980, 149.16507840, 603.7887287},
{-35.36325930, 149.16519780, 603.7887287},
{-35.36326860, 149.16519820, 603.7887287},
{-35.36326710, 149.16507840, 603.7887287},
{-35.36327500, 149.16508360, 603.7887287},
{-35.36327600, 149.16519820, 603.7887287},
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
        set_global_position_and_yaw(target.latitude, target.longitude, target.altitude, nh);

        double current_distance = calculate_distance(current_position.latitude, current_position.longitude, target.latitude, target.longitude);
        ROS_INFO("Current distance to waypoint %lu: %f meters", current_waypoint_index, current_distance);

        // 检查是否到达目标点（例如，距离小于10米）
        if (current_distance < 0.2) {
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