#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <gnc_functions.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <gdp_main/CoordinateList.h>
#include <cmath>

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
float target_alt = 3;

//Check waypoints reached tolerance
float pose_tolerance = 0.3; //metres
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

//Coordinate structure
struct Coordinate 
{
	float x;
	float y;
};

Coordinate getTrackingWaypoint(float current_pos_x, float current_pos_y, float currentHeading, float distanceToCentre_x, float distanceToCentre_y)
{
	Coordinate trackingWaypoint;

	float beta = atan(distanceToCentre_y/distanceToCentre_x); //target angle from x middle axis (camera frame) 
	float gamma = currentHeading - beta;
	float dist2centre = sqrt(pow(distanceToCentre_x, 2) + pow(distanceToCentre_y, 2));


	trackingWaypoint.x = current_pos_x + dist2centre*cos(gamma);
	trackingWaypoint.y = current_pos_y + dist2centre*sin(gamma);

	return trackingWaypoint;
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

//const <msg package name>::<message>::ConstPtr& msg
void darknet_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& darknet_msg) // callback function
{
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

				//Extract the coordinates of the boxes
				int xMin = darknet_msg->bounding_boxes[i].xmin;
				int xMax = darknet_msg->bounding_boxes[i].xmax;
				int yMin = darknet_msg->bounding_boxes[i].ymin;
				int yMax = darknet_msg->bounding_boxes[i].ymax;

				//Print information
				/*ROS_INFO("Boxe number %ld", i);
				ROS_INFO("xmin: %ld", xMin);
				ROS_INFO("xmax: %ld", xMax);
				ROS_INFO("ymin: %ld", yMin);
				ROS_INFO("ymax: %ld", yMax);
				ROS_INFO("id: %ld", darknet_msg->bounding_boxes[i].id);*/

				//Get the center 
				float bb_center_x = xMin + (xMax - xMin) / 2;
				float bb_center_y = yMin + (yMax - yMin) / 2;
				//ROS_INFO("Bounding Box center coordinates: (%lf, %lf)", bb_center_x, bb_center_y);

				//Compute distance to image center in pixel
				float pix_dist2center_x = bb_center_x - img_center_x;
				float pix_dist2center_y = bb_center_y - img_center_y;
				//ROS_INFO("Center is %lf pixels away in x and %lf pixels away in y", pix_dist2center_x, pix_dist2center_y);

				//Convert into meters			
				dist2center_x = pix_dist2center_x * pix2metres; // NEED TO SUBSCRIBE TO PIXEL TO METRES TOPIC 	
				dist2center_y = pix_dist2center_y * pix2metres; // NEED TO SUBSCRIBE TO PIXEL TO METRES TOPIC 
				ROS_INFO("Distance in metres to move in x:%lf and y: %lf", dist2center_x, dist2center_y);
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


// Main function with argument from the launch file 
int main(int argc, char **argv) {

	ros::init(argc, argv, "main"); //name of the node
	ros::NodeHandle n; //enable connection to the ros network

	//	ros::Subscriber sub = <nodehandle>.subscribe("<topic>", <# of msg buffered>, <name of callback function>);
	ros::Subscriber darknet_sub = n.subscribe("/darknet_ros/bounding_boxes", 1, darknet_cb); //1 = how many message buffered. default 1

	//Getting global coordinate
	ros::Subscriber pos_sub = n.subscribe("/mavros/global_position/global", 1, pos_cb);


	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// wait for FCU connection
	// wait4connect();

	//create local reference frame 
	initialize_local_frame();


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0); // loop execution rate
	int counter = 0;
	while(ros::ok()) // loop as long as the node is running
	{	
		if (mode == 0) //SEARCHING MODE
		{	
			ROS_INFO("Searching");
			ros::spinOnce();
		}

		else if (mode == 1) //TRACKING MODE
		{
			ros::spinOnce();
			ROS_INFO("Tracking"); 
			geometry_msgs:: Point current_pos;
			current_pos = get_current_location(); //In regards to the "local frame" created at takeoff
			float current_heading = get_current_heading();

			Coordinate trackingWaypoint;
			trackingWaypoint= getTrackingWaypoint(current_pos.x, current_pos.y, current_heading, dist2center_x, dist2center_y);
			ROS_INFO("Waypoint set to: x:%lf y:%lf", trackingWaypoint.x, trackingWaypoint.y);
		}	
		rate.sleep();
	}

return 0;
return 0;
return 0;
return 0;
}
