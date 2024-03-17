
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

/*void set_global_position_target(double latitude, double longitude, double altitude, ros::NodeHandle& nh) {
    ros::Publisher global_position_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 10);
    
    mavros_msgs::GlobalPositionTarget target;
    target.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
    target.latitude = latitude;
    target.longitude = longitude;
    target.altitude = 585.2;

    global_position_pub.publish(target);
}*/
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
/*    target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ
                        | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ
                        | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;*/
    target.latitude = latitude;
    target.longitude = longitude;
    target.altitude = 585.2;
    target.yaw = M_PI / 2; //  东方的偏航角，以弧度为单位

    /*target.yaw =  0;         north 
      target.yaw =  M_PI;      south
    target.yaw = 3*M_PI/2;     west

    */
    
    position_target_pub.publish(target);
}

/*double takeoff2(double latitude, double longitude, double takeoff_alt, ros::NodeHandle& nh)


 {

    ros::Publisher global_position_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 10);
    
   // 初始化起飞点发布者
    ros::Publisher global_takeoff_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("/mavros/setpoint_position/global", 10);

    // 初始化服务客户端用于解锁（arm）和设置飞行模式
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 构造起飞点
    geographic_msgs::GeoPoseStamped takeoff_target;
    takeoff_target.header.stamp = ros::Time::now();
    takeoff_target.pose.position.latitude = latitude;
    takeoff_target.pose.position.longitude = longitude;
    takeoff_target.pose.position.altitude = takeoff_alt; // 使用参数中的altitude，而不是固定值

    global_position_pub.publish(takeoff_target);

    for(int i=0; i<100; i++)
    {
        local_pos_pub.publish(waypoint_g);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    // arming
    ROS_INFO("Arming drone");
    mavros_msgs::CommandBool arm_request;
    arm_request.request.value = true;
    while (!current_state_g.armed && !arm_request.response.success && ros::ok())
    {
        ros::Duration(.1).sleep();
        arming_client.call(arm_request);
        local_pos_pub.publish(waypoint_g);
    }
    if(arm_request.response.success)
    {
        ROS_INFO("Arming Successful");  
    }else{
        ROS_INFO("Arming failed with %d", arm_request.response.success);
        return -1;  
    }

    //request takeoff
    
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = takeoff_alt;
    if(takeoff_client.call(srv_takeoff)){
        sleep(3);
        ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
        return -2;
    }
    sleep(2);
    return 0; 
}
void set_global_takeoff_target1(double latitude, double longitude, double altitude, ros::NodeHandle& nh) {
    // 初始化起飞点发布者
    ros::Publisher global_takeoff_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("/mavros/setpoint_position/global", 10);

    // 初始化服务客户端用于解锁（arm）和设置飞行模式
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 构造起飞点
    geographic_msgs::GeoPoseStamped takeoff_target;
    takeoff_target.header.stamp = ros::Time::now();
    takeoff_target.pose.position.latitude = latitude;
    takeoff_target.pose.position.longitude = longitude;
    takeoff_target.pose.position.altitude = altitude; // 使用参数中的altitude，而不是固定值

    // 发布起飞点
    for(int i = 0; i < 100; i++) {
        global_takeoff_pub.publish(takeoff_target);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // 设置飞行模式为GUIDED，准备起飞
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "GUIDED";
    if(set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
        ROS_INFO("GUIDED mode set");
    } else {
        ROS_ERROR("Failed to set GUIDED mode");
        return;
    }

    // 解锁（arm）无人机
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    } else {
        ROS_ERROR("Failed to arm vehicle");
        return;
    }

    // 等待几秒确保无人机稳定
    ros::Duration(5.0).sleep();

    // 发送起飞指令
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = altitude;
    takeoff_cmd.request.latitude = latitude;
    takeoff_cmd.request.longitude = longitude;
    if(takeoff_client.call(takeoff_cmd) && takeoff_cmd.response.success) {
        ROS_INFO("Takeoff command sent successfully");
    } else {
        ROS_ERROR("Failed to send takeoff command");
    }
}
*/
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



/*
     //searching are 1 waypoints
  
{-35.3632670, 149.1650882, 603.7887287},
{-35.36325830, 149.16515750, 603.7887287},
{-35.36324170, 149.16507850, 603.7887287},
{-35.36324360, 149.16519760, 603.7887287},
{-35.36325070, 149.16524360, 603.7887287},
{-35.36324990, 149.16507850, 603.7887287},
{-35.36325980, 149.16507840, 603.7887287},
{-35.36325930, 149.16519780, 603.7887287},
{-35.36326860, 149.16519820, 603.7887287},
{-35.36326710, 149.16507840, 603.7887287},
{-35.36327500, 149.16508360, 603.7887287},
{-35.36327600, 149.16519820, 603.7887287},
{-35.36325650, 149.16510530, 603.7887287},
{-35.36325890, 149.16510530, 603.7887287},*/




/*// Search Area 2 


//{-35.3632612, 149.1650909, 603.7887287},
//{-35.36325830, 149.16515750, 603.7887287},
{-35.36324170, 149.16507860, 603.7887287},
{-35.36324240, 149.16520450, 603.7887287},
{-35.36325070, 149.16520450, 603.7887287},
{-35.36324990, 149.16507860, 603.7887287},
{-35.36325980, 149.16507840, 603.7887287},
{-35.36325950, 149.16520410, 603.7887287},
{-35.36326840, 149.16520400, 603.7887287},
{-35.36326710, 149.16507840, 603.7887287},
{-35.36327640, 149.16507860, 603.7887287},
{-35.36327640, 149.16520430, 603.7887287},
//{-35.36325650, 149.16510530, 603.7887287},
//{-35.36325890, 149.16510530, 603.7887287},
*/

     //new geofenceing
 
{-35.3632612, 149.1650909, 603.7887287},
{-35.36325870, 149.16507540, 603.7887287},
{-35.36327510, 149.16507540, 603.7887287},
{-35.36327740, 149.16507870, 603.7887287},
{-35.36327820, 149.16524340, 603.7887287},
{-35.36326520, 149.16525730, 603.7887287},
{-35.36321510, 149.16525790, 603.7887287},
{-35.36321480, 149.16527240, 603.7887287},
{-35.36315890, 149.16527160, 603.7887287},
{-35.36315900, 149.16520600, 603.7887287},
{-35.36324030, 149.16520490, 603.7887287},
{-35.36324020, 149.16507570, 603.7887287},


                        //square mission
    //{-35.363261, 149.16523, 603.7887287},  // 初始位置 
    //{-35.36325199, 149.16523, 603.7887287},  //North 1
   // {-35.36325199, 149.16524100, 603.7887287},  // est 1
    //{-35.36327001, 149.16524100, 603.7887287},  //south 2
   // {-35.36327001, 149.16521899, 603.7887287},  // west 2
    //{-35.36325199, 149.16521899, 603.7887287},  // north 2  
    //{-35.363261, 149.16523, 603.7887287},



    // {-35.36326200, 149.16523700,0 } ,
    // {-35.36317460, 149.16526520, 603.7887287},  // 第六个目标点  
    // {-35.36317510, 149.16522660, 603.7887287},  // 第七个目标点 

    // {-35.3631910, 149.16522660, 603.7887287},  // 第七下1
   //  {-35.3631910, 149.16526520, 603.7887287},  // 第六下1

    // {-35.36321160, 149.16526730, 603.7887287},  // 第五个目标点
    // {-35.36321160, 149.16522660, 603.7887287},  // 第五左

   //  {-35.36324310, 149.16522190, 603.7887287},  // 第八个目标点
   //  {-35.36324310, 149.16525460, 603.7887287},  // 第八向右
   //  {-35.36324400, 149.16508600, 603.7887287},  // 第九个
   //  {-35.36326100, 149.16508600, 603.7887287},  // 第一个目标点
   //  {-35.36326100, 149.16525460, 603.7887287},  //第一右

 //  {-35.36327290, 149.16525380, 603.7887287},  // 第三个目标点
  // {-35.36327800, 149.16508600, 603.7887287},  // 第二个目标点

   // {-35.36326100, 149.16508600, 603.7887287},   第一个目标点
    //{-35.36327800, 149.16508600, 603.7887287},  第二个目标点
   
    //{-35.36321200, 149.16525460, 603.7887287},  第四个目标点
    
   
    
   
    //{-35.36324400, 149.16508600, 603.7887287},9

    // 可以继续添加更多目标点，每个都保持1.5米的高度
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle nh;
  //  initialize_local_frame();

    init_publisher_subscriber(nh);
    
    ros::Subscriber position_sub = nh.subscribe("/mavros/global_position/global", 10, globalPositionCallback);
    
    wait4connect();
    wait4start();
   // takeoff(1.7);
    double latitude1 = -35.363261; // 示例纬度
    double longitude1 = 149.16523; // 示例经度
    double altitude1 = 585.2; // 起飞后的目标高度，单位为米

    takeoff(1.2);
  // takeoff2(latitude1, longitude1,altitude1, nh);
 // set_global_takeoff_target1(latitude1,longitude1,altitude1,nh);
    ros::Rate rate(2.0); // 2 Hz
    size_t current_waypoint_index = 0; // 当前目标点索引

    while(ros::ok() && current_waypoint_index < waypoints.size()) {
        auto& target = waypoints[current_waypoint_index];
        set_global_position_and_yaw(target.latitude, target.longitude, target.altitude, nh);

        double current_distance = calculate_distance(current_position.latitude, current_position.longitude, target.latitude, target.longitude);
        ROS_INFO("Current distance to waypoint %lu: %f meters", current_waypoint_index, current_distance);

        // 检查是否到达目标点（例如，距离小于10米）
        if (current_distance < 0.05) {
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