#include <gnc_functions.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <cmath>

// 起飞点的经纬度常量
const double TAKEOFF_LAT = -35.3631853;
const double TAKEOFF_LON = 149.1652363;

// 将经度和纬度差值转换为米
void latLonToMeters(double lat, double lon, double& x, double& y) {
    const double EARTH_RADIUS = 6378137.0; // 地球半径，单位为米
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double takeoff_lat_rad = TAKEOFF_LAT * M_PI / 180.0;
    double takeoff_lon_rad = TAKEOFF_LON * M_PI / 180.0;

    // 计算纬度和经度的差值
    double delta_lat = lat_rad - takeoff_lat_rad;
    double delta_lon = lon_rad - takeoff_lon_rad;

    // 转换为米
    x = delta_lon * EARTH_RADIUS * cos(takeoff_lat_rad);
    y = delta_lat * EARTH_RADIUS;
}

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    ROS_INFO("Current GPS: [%f, %f, %f]", msg->latitude, msg->longitude, msg->altitude);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");

    init_publisher_subscriber(gnc_node);
    ros::Subscriber position_sub = gnc_node.subscribe("/mavros/global_position/global", 10, globalPositionCallback);

    wait4connect();
    set_mode("GUIDED");
    initialize_local_frame();
    takeoff(3);

    // 目标点列表（经度，纬度，高度）
    std::vector<std::tuple<double, double, double>> targetPoints = {
    {-35.36326100, 149.16508680, 1.5},  // 第一个目标点
    {-35.36327800, 149.16508900, 1.5},  // 第二个目标点
    {-35.36327290, 149.16525380, 1.5},  // 第三个目标点
    {-35.36321200, 149.16525460, 1.5},  // 第四个目标点
    {-35.36321160, 149.16526730, 1.5},  // 第五个目标点
    {-35.36317460, 149.16526520, 1.5},  // 第六个目标点
    {-35.36317510, 149.16522660, 1.5},  // 第七个目标点
    {-35.36324310, 149.16522190, 1.5},  // 第八个目标点
    {-35.36324400, 149.16508600, 1.5},  // 第九个目标点nextWayPoint.psi也需要被设置
    };

    std::vector<gnc_api_waypoint> waypointList;
    for (size_t i = 0; i < targetPoints.size(); ++i) {
        auto& target = targetPoints[i];
        double x, y;
        latLonToMeters(std::get<0>(target), std::get<1>(target), x, y);
        gnc_api_waypoint nextWayPoint;
        nextWayPoint.x = x;
        nextWayPoint.y = y;
        nextWayPoint.z = std::get<2>(target); // 使用目标点的高度
        nextWayPoint.psi = 0; // 根据需要设置psi值，这里暂时设为0
        waypointList.push_back(nextWayPoint);
    }

    ros::Rate rate(2.0);
    int counter = 0;
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        if (check_waypoint_reached(.3) == 1) {
            if (counter < waypointList.size()) {
                set_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);
                counter++;  
            } else {
                land();
                break;
            }   
        }   
    }
    return 0;
}
