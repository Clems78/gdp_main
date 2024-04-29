#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <gnc_functions.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <darknet_ros_msgs/ObjectCount.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/CommandTOL.h>

#include <geographic_msgs/GeoPoseStamped.h>

using namespace std;

/////////////////////////////////////////////// PARAMETERS ///////////////////////////////////////////////

//Image width and height
int width = 640;
int height = 480;
int img_center_x = width/2;
int img_center_y = height/2;

//Flying altitude
float target_alt = 1.2;

//Check waypoints reached tolerance
float pose_tolerance = 0.5; //metres
float heading_tolerance = 7; //degrees

/////////////////////////////////////////////// PARAMETERS ///////////////////////////////////////////////

bool tracking_flag;

// Detection counter
int detection_counter_rt = 0;
int detection_counter = 0;
ros::Time last_zero_detection_time;

// 0 = searching // 1 = tracking
int mode = 0;

//const <msg package name>::<message>::ConstPtr& msg
void yolo_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& yolo_msg) // callback function
{
	// ROS_INFO("Yolo loop");
	// ROS_INFO("mode :  %ld", mode);
	// ROS_INFO("Tracking flag: %s", tracking_flag ? "true" : "false");

	if (tracking_flag == false)
	{
		for (int i=0; i<yolo_msg->bounding_boxes.size(); i++ ) // run on each bouding box that is in the message
		{
			// ROS_INFO("Number of bb detected: %d", yolo_msg->bounding_boxes.size()); //convert into a char array that can be used by ros; print detected object

			string boxe_name = yolo_msg->bounding_boxes[i].Class.c_str();
			// int detection_flag = 0;

			if (boxe_name == "person" ) // 
			{
				mode = 1;
				ROS_INFO("MODE TO 1");
			}
		}
	}
}

//const <msg package name>::<message>::ConstPtr& msg
void object_count_cb(const darknet_ros_msgs::ObjectCount::ConstPtr& object_count_msg) // callback function
{
	detection_counter = object_count_msg->count;
    ROS_INFO("Object counter realtime : %d", detection_counter);
   	ROS_INFO("Tracking flag: %s", tracking_flag ? "true" : "false");

    if (detection_counter != 0 && tracking_flag == true) {
        last_zero_detection_time = ros::Time::now();
    }

     ros::Time current_time = ros::Time::now();
     ros::Duration elapsed_time = current_time - last_zero_detection_time;
 	 ROS_INFO("Time elapsed since last_zero_detection_time: %f seconds", elapsed_time.toSec());

    if (tracking_flag == true && (ros::Time::now() - last_zero_detection_time).toSec() > 3.0)
    {
        tracking_flag = false;
        ROS_INFO("////////////////////Detection enabled again//////////////////:");
    }
}
/////////////////////////////////////////////////////////search everinment/////////////////////////////////////////
sensor_msgs::NavSatFix current_position;

///////////////PARAMETERS///////////////
//float pose_tolerance = 0.2;
///////////////PARAMETERS///////////////

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    current_position = *msg;
   // ROS_INFO("Current GPS: [%f, %f, %f]", msg->latitude, msg->longitude, msg->altitude);
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
    // target.yaw = M_PI / 2;    // est rad
    target.yaw =  0;             // north 
     //target.yaw =  M_PI;       // south
     // target.yaw = 3*M_PI/2;   // west
    
    
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
{-35.36324490, 149.16508210, 603.7887287},
{-35.36324350, 149.16521070, 603.7887287},
{-35.36325180, 149.16521070, 603.7887287},
{-35.36325140, 149.16508160, 603.7887287},
{-35.36326030, 149.16508160, 603.7887287},
{-35.36326080, 149.16521080, 603.7887287},
{-35.36326860, 149.16521050, 603.7887287},
{-35.36326780, 149.16508240, 603.7887287},
{-35.36327470, 149.16508260, 603.7887287},
{-35.36327480, 149.16521070, 603.7887287},
};


int main(int argc, char **argv) {

	ros::init(argc, argv, "main"); //name of the node
	ros::NodeHandle n; //enable connection to the ros network

	//	ros::Subscriber sub = <nodehandle>.subscribe("<topic>", <# of msg buffered>, <name of callback function>);
	ros::Subscriber yolo_sub = n.subscribe("/darknet_ros/bounding_boxes", 10, yolo_cb); //1 = how many message buffered. default 1

	ros::Subscriber object_count_sub = n.subscribe("/darknet_ros/found_object", 1, object_count_cb); //1 = how many message buffered. default 1

	//initialize control publisher/subscribers
	init_publisher_subscriber(n);
    ros::Subscriber position_sub = n.subscribe("/mavros/global_position/global", 10, globalPositionCallback);
  	// wait for FCU connection
    wait4connect();
     
	//create local reference frame 
	initialize_local_frame();
    wait4start();
    takeoff(1.2);
	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(5.0); // loop execution rate

	size_t current_waypoint_index = 0; // 当前目标点索引

	 while(ros::ok() && current_waypoint_index < waypoints.size())  // loop as long as the node is running
	{	
		// ROS_INFO("MAIN LOOP");

		if (mode == 0) //SEARCHING MODE
		{	
		 	ROS_INFO("Searching");
	    auto& target = waypoints[current_waypoint_index];
        set_global_position_and_yaw(target.latitude, target.longitude, target.altitude, n);

        double current_distance = calculate_distance(current_position.latitude, current_position.longitude, target.latitude, target.longitude);
        // ROS_INFO("Current distance to waypoint %lu: %f meters", current_waypoint_index, current_distance);

        // 检查是否到达目标点（例如，距离小于10米）
        if (current_distance < pose_tolerance) {
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

		else if (mode == 1) //TRACKING MODE
		{

        
			ROS_INFO("starting delay");
	
    
			float currentLatitude = current_position.latitude;
			float currentLongitude = current_position.longitude;


            set_global_position_and_yaw(currentLatitude, currentLongitude, 1.2, n);
            double current_distance = calculate_distance(current_position.latitude, current_position.longitude, currentLatitude, currentLongitude);
           // ROS_INFO("Current distance to waypoint: %f meters", current_distance);
			
			tracking_flag = true;
			ROS_INFO("MODE TO 0");
		    if (current_distance < 1) { // 到达阈值
                ROS_INFO("Arrived at waypoint");
               // t222 = false; // 防止重复执行
                mode = 0;
            ros::Duration delay(5.0);
			delay.sleep();
		                    
		     }	

		rate.sleep();
		ros::spinOnce();
	}
}
return 0;
}
