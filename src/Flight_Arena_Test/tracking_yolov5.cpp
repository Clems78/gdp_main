#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <gnc_functions.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <gdp_main/CoordinateList.h>
#include <cmath>

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

//Distance to centres
float dist2center_x;
float dist2center_y;

// 0 = searching // 1 = tracking
int mode = 0;

//Coordinate structure
struct Coordinate 
{
	float x;
	float y;
};

// Initialise the coordinate of the tracked vehicle 
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


//const <msg package name>::<message>::ConstPtr& msg
void yolo_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& yolo_msg) // callback function
{
	ROS_INFO("Yolo loop");
	// ROS_INFO("mode :  %ld", mode);

	for (int i=0; i<yolo_msg->bounding_boxes.size(); i++ ) // run on each bouding box that is in the message
	{
		// ROS_INFO("%s detected", yolo_msg->bounding_boxes[i].Class.c_str()); //convert into a char array that can be used by ros; print detected object

		string boxe_name = yolo_msg->bounding_boxes[i].Class.c_str();
		// int detection_flag = 0;

		if (boxe_name == "car") // 
		{
				// ROS_INFO("%s detected", yolo_msg->bounding_boxes[i].Class.c_str());
				// ROS_INFO("detection flag in darknet_ros: %ld", detection_flag);
				mode = 1;

				//Extract the coordinates of the boxes
				int xMin = yolo_msg->bounding_boxes[i].xmin;
				int xMax = yolo_msg->bounding_boxes[i].xmax;
				int yMin = yolo_msg->bounding_boxes[i].ymin;
				int yMax = yolo_msg->bounding_boxes[i].ymax;

				//Get the center of the bounding boxes
				float bb_center_x = xMin + (xMax - xMin) / 2;
				float bb_center_y = yMin + (yMax - yMin) / 2;
				//ROS_INFO("Bounding Box center coordinates: (%lf, %lf)", bb_center_x, bb_center_y);

				//Compute distance between center of bounding boxed and image center in pixel
				float pix_dist2center_x = bb_center_x - img_center_x;
				float pix_dist2center_y = bb_center_y - img_center_y;
				//ROS_INFO("Center is %lf pixels away in x and %lf pixels away in y", pix_dist2center_x, pix_dist2center_y);

				// ros::Time timestamp = yolo_msg->header.stamp;
			    // Print the timestamp
			    // ROS_INFO("Timestamp of bounding box message: %f", timestamp.toSec());

				// Get current location of the drone
				geometry_msgs::Point current_pos;
				current_pos = get_current_location(); //In regards to the "local frame" created at takeoff	
				// ROS_INFO("current position x: %lf y:%lf z:%lf",current_pos.x, current_pos.y, current_pos.z);

				//Get current heading 
				float current_heading = get_current_heading();
				// ROS_INFO("Current heading: %lf", current_heading);			

				//Convert into metres
				P2M p2m = getPix2Metres(current_pos.z);
				double dist2center_x = pix_dist2center_x * p2m.x ; // NEED TO SUBSCRIBE TO PIXEL TO METRES TOPIC 	
				double dist2center_y = pix_dist2center_y * p2m.y; // NEED TO SUBSCRIBE TO PIXEL TO METRES TOPIC 
				ROS_INFO("Distance to center in x :%lf and y: %lf", dist2center_x, dist2center_y);			

				//Convert from camera frame to global frame
				trackingWaypoint = localToGlobal(current_pos.x, current_pos.y, current_heading * M_PI/180, dist2center_x, dist2center_y);
				// ROS_INFO("Waypoint coordinate in x:%lf and y: %lf", trackingWaypoint.x, trackingWaypoint.y);
		}
	}
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "main"); //name of the node
	ros::NodeHandle n; //enable connection to the ros network

	//	ros::Subscriber sub = <nodehandle>.subscribe("<topic>", <# of msg buffered>, <name of callback function>);
	ros::Subscriber yolo_sub = n.subscribe("/bounding_boxes", 10, yolo_cb); //1 = how many message buffered. default 1
	// AS long as there is no new messages in this topic, the callback function is not entered

	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// wait for FCU connection
	wait4connect();

	//create local reference frame 
	initialize_local_frame();


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(5.0); // loop execution rate

	while(ros::ok()) // loop as long as the node is running
	{	ROS_INFO("MAIN LOOP");

		if (mode == 0) //SEARCHING MODE
		{	
			ROS_INFO("Searching");
			ros::spinOnce();
		}

		else if (mode == 1) //TRACKING MODE
		{
			// ros::spinOnce();
			ROS_INFO("Tracking"); 
		
			if(check_waypoint_reached(pose_tolerance, heading_tolerance) == 1)
			{	
				// 	ros::Duration delay(10.0);
			    // Sleep for the specified duration
			    // delay.sleep();
				set_destination(trackingWaypoint.x, trackingWaypoint.y, target_alt, 0); //Make the drone move to the target location
			}
			
		}	
		rate.sleep();
		ros::spinOnce();
	}
return 0;
}
