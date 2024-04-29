#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <gnc_functions.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <gdp_main/CoordinateList.h>
#include <cmath>
//#include "gnc_functions2.hpp"
using namespace std;

/////////////////////////////////////////////// PARAMETERS ///////////////////////////////////////////////

//Re tracking distance threshold
float tracking_coord_threshold = 0.001;

//Image width and height
int width = 640;
int height = 480;
int img_center_x = width/2;
int img_center_y = height/2;
struct tg2 {
    double latitude;
    double longitude;
    double altitude;
};
//Pixel to metres topic 

float pix2metres = 0.00332812; // Subscribes to pixel to metres topic // Need another callback function

// Time after which the drone goes back to searching if the target is lost during tracking 
int lost_target_time = 5;

//Tracking time
int tracking_time = 10;

//Flying altitude
float target_alt = 1.2;

//Check waypoints reached tolerance
float pose_tolerance = 0.2; //metres
float heading_tolerance = 2; //degrees

/////////////////////////////////////////////// PARAMETERS ///////////////////////////////////////////////

//Distance to centres
float dist2center_x;
float dist2center_y;

// 0 = searching // 1 = tracking
int mode = 0;

//Target lost counter
int target_lost_counter = 0;

//Tracking counter
int tracking_counter = 0;

//Condition on first tracking waypoint
int first_tracking_waypoint = 0;

//Detection flag
int detection_flag = 0;

//Publisher//
//tracked vehicle coordinate publisher 
ros::Publisher tracked_vehicle_pos_pub;

//Initialise tracked vehicle coordinate message
geometry_msgs::Point position_msg;



//subscriber
// ros::Subscriber currentPos_test;
// nav_msgs::Odometry current_pose_g_test;

//Altitude local frame
float alt_local;

//Coordinate structure
struct Coordinate 
{
	float x;
	float y;
};

//Coordinate tracking waypoint
Coordinate trackingWaypoint;

struct P2M
{
	double x; 
	double y;
};


// Assuming the angle is provided in radians
Coordinate localToGlobal(float drone_x, float drone_y, float drone_heading, double target_local_x, double target_local_y) {
    // Rotation matrix for transforming local coordinates to global coordinates
    float cos_heading = cos(drone_heading);
    float sin_heading = sin(drone_heading);

    float target_global_x = drone_x + cos_heading * target_local_x + sin_heading * target_local_y;
    float target_global_y = drone_y + sin_heading * target_local_x - cos_heading * target_local_y;

    Coordinate target_global = {target_global_x, target_global_y};
    // ROS_INFO("Waypoint is x: %lf and y: %lf", target_global_x, target_global_y);
    return target_global;
}

P2M getPix2Metres(float altitude) //altitude in metres
{
	double pix2metres_x = 0.00045312 * altitude + 0.00277604;	
	double pix2metres_y = 0.00068750 * altitude + 0.00238889;

	P2M pix2metres = {pix2metres_x, pix2metres_y};
	
	return pix2metres;
}

//Get global coordinates of the drone
void pos_cb(const sensor_msgs::NavSatFix::ConstPtr& pos_msg)
{
	//Extract data
	float latitude = pos_msg->latitude;
	float longitude = pos_msg->longitude;
	float altitude = pos_msg->altitude;

	//Detail message content
	position_msg.x = latitude;
    position_msg.y = longitude;
    position_msg.z = altitude;
}

// //Get local coordinates of the drone
// void loc_pos_cb(const nav_msgs::Odometry::ConstPtr& loc_pos_msg)
// {
// 	//Extract data
// 	alt_local = loc_pos_msg->pose.pose.position.z;

// }
bool person_detected = false;

//const <msg package name>::<message>::ConstPtr& msg
void darknet_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& darknet_msg)
{
    ROS_INFO("Processing darknet bounding boxes...");

    bool person_detected = false;

    for (const auto& box : darknet_msg->bounding_boxes) {
        if (box.Class == "person") {
            ROS_INFO("%s detected", box.Class.c_str());
            person_detected = true;
            target_lost_counter = 0; // Reset the counter since we've found our target
            break; // Exit the loop early since we've found what we're looking for
        }
        else
        {
        person_detected = false;
        break;	
        }
    }

   /* if (person_detected) {
        mode = 1; // Switch to tracking mode
    } else {
        ROS_INFO("No person detected, switching to search mode.");
        mode = 0; // Switch back to search mode
    }
*/
    // You might want to handle the target_lost_counter logic outside of this callback
    // For example, you could use a separate timer to increment the counter and check the mode
}


// void pose_cb_test(const nav_msgs::Odometry::ConstPtr& msg)
// {
//   current_pose_g_test = *msg;
//   enu_2_local(current_pose_g_test);
//   float q0 = current_pose_g_test.pose.pose.orientation.w;
//   float q1 = current_pose_g_test.pose.pose.orientation.x;
//   float q2 = current_pose_g_test.pose.pose.orientation.y;
//   float q3 = current_pose_g_test.pose.pose.orientation.z;
//   float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
//   //ROS_INFO("Current Heading %f ENU", psi*(180/M_PI));
//   //Heading is in ENU
//   //IS YAWING COUNTERCLOCKWISE POSITIVE?
//   current_heading_g = psi*(180/M_PI) - local_offset_g;
//   //ROS_INFO("Current Heading %f origin", current_heading_g);
//   //ROS_INFO("x: %f y: %f z: %f", current_pose_g_test.pose.pose.position.x, current_pose_g_test.pose.pose.position.y, current_pose_g_test.pose.pose.position.z);
// }

// Main function with argument from the launch file 
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
/*    target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ
                        | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ
                        | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;*/
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
  
int current_waypoint_index = 0;



tg2 target2;



tg2 last_target = {0, 0, 0}; // 上一个目标位置
bool update_target2 = true; // 是否更新目标点的标志
bool t222 = true; // 额外的逻辑控制变量，根据你的需求进行调整

int main(int argc, char **argv) {
    
	ros::init(argc, argv, "main"); //name of the node
	ros::NodeHandle n; //enable connection to the ros network
   // ros::NodeHandle nh;
    //init_publisher_subscriber(nh);
    ros::Subscriber position_sub = n.subscribe("/mavros/global_position/global", 10, globalPositionCallback);
	ros::Subscriber darknet_sub = n.subscribe("/darknet_ros/bounding_boxes", 1, darknet_cb); //1 = how many message buffered. default 1
	//ros::Subscriber pos_sub = n.subscribe("/mavros/global_position/global", 1, pos_cb);
	init_publisher_subscriber(n);

	wait4connect();
    set_mode("GUIDED");
	//create local reference frame 
	initialize_local_frame();
    takeoff(1.2);
	set_speed(1);


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0); // loop execution rate
	int counter = 0;
	 while (ros::ok()) {
        switch (mode) {
            case 0: { // Searching mode
                ROS_INFO("Searching");
                if (current_waypoint_index < waypoints.size()) {
                    auto& target = waypoints[current_waypoint_index];
                    set_global_position_and_yaw(target.latitude, target.longitude, target.altitude, n);

                    double current_distance = calculate_distance(current_position.latitude, current_position.longitude, target.latitude, target.longitude);
                    ROS_INFO("Current distance to waypoint %zu: %f meters", current_waypoint_index, current_distance);

                    if (current_distance < 1) { // Threshold distance to waypoint
                        ROS_INFO("Arrived at waypoint %zu.", current_waypoint_index++);
                        if (current_waypoint_index >= waypoints.size()) {
                            ROS_INFO("All waypoints reached. Preparing to land.");
                            land();
                            return 0; // End the program after landing
                        }
                    }
                }
                break;
            }

             case 1: {
        ROS_INFO("Tracking");
        if (update_target2) {
            tg2 new_target = {current_position.latitude, current_position.longitude, 1.2};
            double distance_to_last_target = calculate_distance(new_target.latitude, new_target.longitude, last_target.latitude, last_target.longitude);

            const double DISTANCE_THRESHOLD = 1.0; // 10米阈值
            if (distance_to_last_target > DISTANCE_THRESHOLD) {
                target2 = new_target;
                last_target = new_target;
                update_target2 = false; // 已更新
            }
        }
        
        if(t222 && !update_target2) {
            set_global_position_and_yaw(target2.latitude, target2.longitude, 1.2, n);
            double current_distance = calculate_distance(current_position.latitude, current_position.longitude, target2.latitude, target2.longitude);
            ROS_INFO("Current distance to waypoint: %f meters", current_distance);

            if (current_distance < 1) { // 到达阈值
                ROS_INFO("Arrived at waypoint");
               // t222 = false; // 防止重复执行
                break; // 结束当前case的执行
            }
        } else {
            ROS_INFO("waypoint back");
           // mode = 0; // 更改模式
        }
        break; // 结束case 1
    }
            
                
                // This is where you would adjust the drone's pobreaksition based on the tracking data.
                break;
            }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}