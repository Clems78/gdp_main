
#include <thread>
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <cmath>
#include <chrono>
#include <atomic>
#include <mavros_msgs/State.h>

// 假设这个头文件提供了所需的辅助函数如 init_publisher_subscriber(), wait4connect(), wait4start(), takeoff(), land()

#include <gnc_functions.hpp>

sensor_msgs::NavSatFix current_position;
std::atomic<bool> drone2_landed(false);


void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    current_position = *msg;
    ROS_INFO("Current GPS: [%f, %f, %f]", msg->latitude, msg->longitude, msg->altitude);
}


void set_global_position_and_yaw(double latitude, double longitude, double altitude, ros::NodeHandle& nh) {
    ros::Publisher position_target_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 10);

    // 确保发布者有时间来连接到ROS主题
    ros::Rate rate(20);
    while (position_target_pub.getNumSubscribers() < 1) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::GlobalPositionTarget target;
    target.header.stamp = ros::Time::now();
    target.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
/*  target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ
                        | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ
                        | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;*/
    target.latitude = latitude;
    target.longitude = longitude;
    target.altitude = 585.2;
    target.yaw = M_PI / 2;    //  est rad
    //target.yaw =  0;        // north 
    //target.yaw =  M_PI;     // south
    //target.yaw = 3*M_PI/2;  //  west


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





std::vector<TargetPoint> waypoints = {




     //searching are 1 waypoints
  
//{-35.3632670, 149.1650882, 603.7887287},
//{-35.36325830, 149.16515750, 603.7887287},

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

//{-35.36325650, 149.16510530, 603.7887287},
//{-35.36325890, 149.16510530, 603.7887287},



};


std::vector<TargetPoint> waypoints2 = {


// Search Area 2 


 //{-35.3632612, 149.1650909, 603.7887287},
//{-35.36325830, 149.16515750, 603.7887287},
{-35.36327360, 149.16521110, 603.7887287},
{-35.36316200, 149.16521180, 603.7887287},
{-35.36316200, 149.16522260, 603.7887287},
{-35.36327380, 149.16522140, 603.7887287},
{-35.36327340, 149.16523160, 603.7887287},
{-35.36316230, 149.16523120, 603.7887287},
{-35.36316260, 149.16524200, 603.7887287},
{-35.36327210, 149.16524050, 603.7887287},
{-35.36326250, 149.16525040, 603.7887287},
{-35.36316170, 149.16525160, 603.7887287},
{-35.36316200, 149.16526350, 603.7887287},
{-35.36321050, 149.16526300, 603.7887287},
//{-35.36325830, 149.16509110, 603.7887287},
//{-35.36326070, 149.16509080, 603.7887287},

};

std::vector<TargetPoint> waypoints13 = {


// Search Area 2


 //{-35.3632612, 149.1650909, 603.7887287},
//{-35.36325830, 149.16515750, 603.7887287},
{-35.36327360, 149.16521110, 603.7887287},
{-35.36316200, 149.16521180, 603.7887287},
{-35.36316200, 149.16522260, 603.7887287},
{-35.36327380, 149.16522140, 603.7887287},
{-35.36327340, 149.16523160, 603.7887287},
{-35.36316230, 149.16523120, 603.7887287},
{-35.36316260, 149.16524200, 603.7887287},
{-35.36327210, 149.16524050, 603.7887287},
{-35.36326250, 149.16525040, 603.7887287},
{-35.36316170, 149.16525160, 603.7887287},
{-35.36316200, 149.16526350, 603.7887287},
{-35.36321050, 149.16526300, 603.7887287},
//{-35.36325830, 149.16509110, 603.7887287},
//{-35.36326070, 149.16509080, 603.7887287},


};

std::vector<TargetPoint> waypoints23 = {


// Search Area 2


     //searching are 1 waypoints
  
//{-35.3632670, 149.1650882, 603.7887287},
//{-35.36325830, 149.16515750, 603.7887287},
{-35.36321050, 149.16523120, 603.7887287}, 
{-35.36324170, 149.16523120, 603.7887287},   

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

//{-35.36325650, 149.16510530, 603.7887287},
//{-35.36325890, 149.16510530, 603.7887287},



};

void drone2StateCallback(const mavros_msgs::State::ConstPtr& msg) {
    // 根据状态消息判断无人机2是否已降落的逻辑
    // 根据你表示降落的方式调整这里的逻辑
    if (msg->armed == false) { // 假设通过AUTO.LAND模式或不处于armed状态来表示降落
        drone2_landed.store(true);
    } else {
        drone2_landed.store(false); //应该是false
    }
}

void droneMission(const std::string& drone_ns, std::vector<TargetPoint>& waypoints, std::vector<TargetPoint>& waypoints2) {
    ros::NodeHandle nh(drone_ns);
    ros::NodeHandle nn("/drone1");
    init_publisher_subscriber(nh);

    ros::Subscriber position_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, globalPositionCallback);
    ros::Subscriber state_sub = nn.subscribe<mavros_msgs::State>("mavros/state", 10, drone2StateCallback); // 订阅无人机的状态
    wait4connect();

    set_mode("GUIDED");

    takeoff(1.2);

    ros::Rate rate(2.0);
    size_t current_waypoint_index = 0;
    bool waypoints2_added = false; // 用于标记是否已追加waypoints2

    while (ros::ok() && current_waypoint_index < waypoints.size()) {
        auto& target = waypoints[current_waypoint_index];
        set_global_position_and_yaw(target.latitude, target.longitude, target.altitude, nh);

        double current_distance = calculate_distance(current_position.latitude, current_position.longitude, target.latitude, target.longitude);
        ROS_INFO("[%s] Current distance to waypoint %lu: %f meters", drone_ns.c_str(), current_waypoint_index, current_distance);

        if (current_distance < 0.5) {
            ROS_INFO("[%s] Arrived at waypoint %lu.", drone_ns.c_str(), current_waypoint_index);
            current_waypoint_index++;

            // 当航点数量大于3时检查无人机2的降落状态
            if (current_waypoint_index > 3 && !waypoints2_added) {
                if (drone2_landed.load()) {
                    ROS_INFO("[%s] Drone 2 has landed. Appending additional waypoints.", drone_ns.c_str());
                    waypoints.insert(waypoints.end(), waypoints2.begin(), waypoints2.end());
                    waypoints2_added = true; // 标记waypoints2已被追加
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    // 在完成所有航点后，切换到LOITER模式以悬停
    if (current_waypoint_index >= waypoints.size()) {
        ROS_INFO("[%s] All waypoints reached. Switching to LOITER mode for hovering.", drone_ns.c_str());
        
        ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        mavros_msgs::SetMode set_mode_srv;
        set_mode_srv.request.custom_mode = "LOITER";  // 设置为LOITER模式
        
        if (set_mode_client.call(set_mode_srv) && set_mode_srv.response.mode_sent) {
            ROS_INFO("LOITER mode set successfully.");
        } else {
            ROS_ERROR("Failed to set LOITER mode.");
        }
    }
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "gnc_node_drone2");

    droneMission("/drone2", waypoints2,waypoints23);

    return 0;
}

 // 使用 std::thread 为 drone1 和 drone2 启动任务
   //droneMission("/drone2", waypoints2);

    // 等待线程结束
    // drone1_thread.join();
    // drone2_thread.join();
/*int main(int argc, char **argv) {
    ros::init(argc, argv, "gnc_node_drone1_drone2");

    // 使用 std::thread 为 drone1 和 drone2 启动任务
    std::thread drone1_thread([&]() { droneMission("/drone1", waypoints); });
    std::thread drone2_thread([&]() { droneMission("/drone2", waypoints2); });

    // 等待线程结束
    drone1_thread.join();
    drone2_thread.join();

    return 0;
}*/

/*int main(int argc, char **argv) {

    // 初始化ROS节点

    ros::init(argc, argv, "gnc_node_drone1_drone2");
    
    // 启动2个线程的异步旋转器
    ros::AsyncSpinner spinner(2); 
    spinner.start();
    
    // 启动无人机控制任务的线程
    std::thread drone1_thread(droneMission, "/drone1", waypoints);
    std::thread drone2_thread(droneMission, "/drone2", waypoints2);
    
    // 等待无人机控制任务线程结束
    drone1_thread.join();
    drone2_thread.join();
    
    // 等待ROS的异步旋转器结束
    ros::waitForShutdown();
    
    return 0;
}*/

/*int main(int argc, char **argv) {
    ros::init(argc, argv, "gnc_node_drone1_drone2");

    std::thread drone1_thread([&](){ 
        droneMission("/drone1", waypoints, drone1_done); 
    });
    std::thread drone2_thread([&](){ 
        droneMission("/drone2", waypoints2, drone2_done); 
    });

    drone1_thread.detach();
    drone2_thread.detach();

    // 等待线程结束的简单方式，实际项目中可能需要更复杂的逻辑来确保所有资源的安全释放
    while (!drone1_done || !drone2_done) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    ROS_INFO("All drones have completed their missions.");

    return 0;
}*/