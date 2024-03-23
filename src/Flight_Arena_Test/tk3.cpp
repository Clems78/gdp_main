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

//const <msg package name>::<message>::ConstPtr& msg
void darknet_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& darknet_msg) // callback function
{
	ROS_INFO("darknet loop");
	// ROS_INFO("mode :  %ld", mode);

	for (int i=0; i<darknet_msg->bounding_boxes.size(); i++ ) // run on each bouding box that is in the message
	{
		// ROS_INFO("%s detected", darknet_msg->bounding_boxes[i].Class.c_str()); //convert into a char array that can be used by ros; print detected object

		string boxe_name = darknet_msg->bounding_boxes[i].Class.c_str();
		// int detection_flag = 0;

		if (boxe_name == "person") // condition should be changed to "rccars" We could also restrict the detection of Yolo to only rc cars and remove all the other classes to prevent issues
		{
				ROS_INFO("%s detected", darknet_msg->bounding_boxes[i].Class.c_str());
				ROS_INFO("detection flag in darknet_ros: %ld", detection_flag);
				mode = 1;
				target_lost_counter = 0; // put back target lost counter to 0		    
				
		}

		//Condition for going back to search mode after a certain amount of time if the target is lost ie box_name = empty
		if (mode == 1 && boxe_name == "")
		{
			if (target_lost_counter < lost_target_time)
			{
				//Rate for target lost
				ros::Rate target_lost_rate(1);
				while (ros::ok())
				{
					ROS_INFO("Target lost ! Back to searching in %ld seconds", lost_target_time - target_lost_counter);
					target_lost_counter++;
					target_lost_rate.sleep(); //Ensure that the loop runs once per second
					break;
				}
			}
			else
			{
				ROS_INFO("Target lost ! Back to searching !");
				mode = 0;
			}	
		}	
	}
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

int main(int argc, char **argv) {

	ros::init(argc, argv, "main"); //name of the node
	ros::NodeHandle n; //enable connection to the ros network
    ros::NodeHandle nh;

    init_publisher_subscriber(nh);
    
    ros::Subscriber position_sub = nh.subscribe("/mavros/global_position/global", 10, globalPositionCallback);
    
	//	ros::Subscriber sub = <nodehandle>.subscribe("<topic>", <# of msg buffered>, <name of callback function>);
	ros::Subscriber darknet_sub = n.subscribe("/darknet_ros/bounding_boxes", 1, darknet_cb); //1 = how many message buffered. default 1
	// AS long as there is no new messages in this topic, the callback function is not entered

	//Getting global coordinate
	ros::Subscriber pos_sub = n.subscribe("/mavros/global_position/global", 1, pos_cb);

	//Getting local coordinate
	// ros::Subscriber loc_pos_sub = n.subscribe("/mavros/global_position/local", 1, loc_pos_cb);

	// currentPos_test = controlnode.subscribe<nav_msgs::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, pose_cb_test);


	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// wait for FCU connection
	wait4connect();
    set_mode("GUIDED");
	//create local reference frame 
	initialize_local_frame();

	//wait4start();
    takeoff(1.2);
	//Set speed
	set_speed(1);


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0); // loop execution rate
	int counter = 0;
	while(ros::ok()) // loop as long as the node is running
	{	
		// float current_heading = get_current_heading();
		// // ROS_INFO("Current headind %lf", current_heading_g);

		// geometry_msgs::Point current_location; 
		// current_location = get_current_location();
		// // ROS_INFO("Current location %lf", current_location.x);
       
		if (mode == 0) //SEARCHING MODE
		{	
			ROS_INFO("Searching");

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
			// ros::spinOnce();
		}

		else if (mode == 1) //TRACKING MODE
		{
			// ros::spinOnce();
			ROS_INFO("Tracking"); 
            
            
			// ROS_INFO("Current headind %lf", current_heading_g);

			//Should only be trigered if the bounding boxe is detected
			// Otherwise the waypoints keep being updated with the last position of the drone and bounding boxe
			// Can be put inside the callback function maybe ? 
			// The update rate of the darknet ros is very slow that's a reason for this issue. 
			// But it shows that if the bounding box is lost, the drones flies away. It should stop after reaching the next waypoint
			
			if(check_waypoint_reached(pose_tolerance, heading_tolerance) == 1)
			{	
				ros::Duration delay(5.0);
			    // Sleep for the specified duration
			    delay.sleep();
				set_destination(trackingWaypoint.x, trackingWaypoint.y, target_alt, 0);
				ROS_INFO("Waypoint set to: x:%lf y:%lf", trackingWaypoint.x, trackingWaypoint.y);
			}
			
		}	
		rate.sleep();
		ros::spinOnce();
	}

return 0;
}
