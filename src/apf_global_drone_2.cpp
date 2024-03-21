#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <cmath>
#include <gnc_functions.hpp>

sensor_msgs::NavSatFix current_position_drone1;
sensor_msgs::NavSatFix current_position_drone2;

void globalPositionCallbackDrone1(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    current_position_drone1 = *msg;
    // ROS_INFO("Current GPS Drone 1: [%f, %f, %f]", msg->latitude, msg->longitude, msg->altitude);
}

void globalPositionCallbackDrone2(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    current_position_drone2 = *msg;
    // ROS_INFO("Current GPS Drone 2: [%f, %f, %f]", msg->latitude, msg->longitude, msg->altitude);
}

void set_global_position_target(double latitude, double longitude, double altitude, ros::NodeHandle& nh) {
    ros::Publisher global_position_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("/drone2/mavros/setpoint_raw/global", 10);

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
    {-35.3633, 149.16523, 603.7887287},
    {-35.363261, 149.16523, 603.7887287},       // 初始位置  //North 1
};

double calculate_repulsive_force(double distance) {
    const double k = 1.0;
    const double min_distance = 10.0; // Desired minimum distance between drones
    double force = 0.0;

    if (distance < min_distance) {
        force = k * ((1.0 / distance) - (1.0 / min_distance));
    }

    return force;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_2");
    ros::NodeHandle nh("~");
    initialize_local_frame();

    init_publisher_subscriber(nh);

    ros::Subscriber position_sub_drone1 = nh.subscribe("/drone1/mavros/global_position/global", 10, globalPositionCallbackDrone1);
    ros::Subscriber position_sub_drone2 = nh.subscribe("/drone2/mavros/global_position/global", 10, globalPositionCallbackDrone2);

    wait4connect();
    wait4start();

    takeoff(1.2);

    ros::Rate rate(2.0); // 2 Hz
    size_t current_waypoint_index = 0; // 当前目标点索引

    while(ros::ok() && current_waypoint_index < waypoints.size()) {
        auto& target = waypoints[current_waypoint_index];

        double distance_between_drones = calculate_distance(current_position_drone1.latitude, current_position_drone1.longitude, current_position_drone2.latitude, current_position_drone2.longitude);
        double repulsive_force = calculate_repulsive_force(distance_between_drones);

        double new_latitude = target.latitude + repulsive_force * (target.latitude - current_position_drone2.latitude) / distance_between_drones;
        double new_longitude = target.longitude + repulsive_force * (target.longitude - current_position_drone2.longitude) / distance_between_drones;

        set_global_position_target(new_latitude, new_longitude, target.altitude, nh);

        double current_distance = calculate_distance(current_position_drone1.latitude, current_position_drone1.longitude, target.latitude, target.longitude);

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
