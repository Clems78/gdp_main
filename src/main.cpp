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
int detection_flag;

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

float calculateDistance(float trackedCoordLat, float trackedCoordLong, float droneLat, float droneLong)
{
	return sqrt(pow(droneLat - trackedCoordLat, 2) + pow(droneLong - trackedCoordLong, 2));
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
				detection_flag = 1;
				//mode = 1;
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

void already_tracked_cb(const gdp_main::CoordinateList::ConstPtr& already_tracked_msg)
{
	ROS_INFO("already tracked loop entered");
	ROS_INFO("Detection flag %ld", detection_flag);
	if (detection_flag == 1) // If target detect box detected 
	{
		//get current position in global coordinate
		float drone_lat = position_msg.x; 
		float drone_long = position_msg.y;

		//Boolean condition
		bool allDistanceAboveThreshold = true;

		for (int i=0; i<already_tracked_msg->points.size(); i++)
		{
			float already_tracked_x = already_tracked_msg->points[i].x;
			float already_tracked_y = already_tracked_msg->points[i].y;

			float distanceToCoord = calculateDistance(already_tracked_x, already_tracked_y, drone_lat, drone_long);

			if (distanceToCoord < tracking_coord_threshold)
			{
				allDistanceAboveThreshold = false;
				ROS_INFO("Target already tracked !");
			}
		}

		if (allDistanceAboveThreshold = true)
		{
			mode = 1; //Set to tracking 
			ROS_INFO("New target, commencing tracking !");
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

	//Getting coordinate of already tracked vehicle 
	ros::Subscriber already_tracked_sub = n.subscribe("/all_tracked_vehicle_coords", 1, already_tracked_cb);

	// ros::Publisher pub = n.advertise<std_msgs::message-type>("created topic_name", 5);
	// Publish the position of the tracked vechile on another node
	tracked_vehicle_pos_pub = n.advertise<geometry_msgs::Point>("tracked_vehicle_pos", 1); //1000 is the queue size


	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// wait for FCU connection
	wait4connect();

	//Wait for user to set mode to guided
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(target_alt);

	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = target_alt;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 0;
	nextWayPoint.z = target_alt;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 5;
	nextWayPoint.z = target_alt;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 5;
	nextWayPoint.z = target_alt;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = target_alt;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);

	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0); // loop execution rate
	int counter = 0;
	while(ros::ok()) // loop as long as the node is running
	{	
		ROS_INFO("");
		if (mode == 0) //SEARCHING MODE
		{	
			geometry_msgs::Point current_pos_2;
			current_pos_2 = get_current_location();
			ROS_INFO("Current position", current_pos_2.x);
			ros::spinOnce(); // Allows to continue processing main loop but still using callback information
			ROS_INFO("#####SEARCHING#####SEARCHING#####SEARCHING#####");
			tracking_counter = 0;
			first_tracking_waypoint = 0;
			rate.sleep(); // Ensure ros rate 
			if(check_waypoint_reached(pose_tolerance, heading_tolerance) == 1)
			{
				if (counter < waypointList.size())
				{	ROS_INFO("Going to next searching point ");
					set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
					counter++;	
				}
			}	
		}

		else if (mode == 1) //TRACKING MODE
		{
			ros::spinOnce();
			ROS_INFO("#####TRACKING#####TRACKING#####TRACKING#####"); 
			geometry_msgs:: Point current_pos;
			current_pos = get_current_location(); //In regards to the "local frame" created at takeoff
			float current_heading = get_current_heading();

			if(first_tracking_waypoint == 0)
			{
				//ROS_INFO("Current position : %lf, %lf", current_pos.x, current_pos.y);
				Coordinate firstTrackingWaypoint;
				firstTrackingWaypoint = getTrackingWaypoint(current_pos.x, current_pos.y, current_heading, dist2center_x, dist2center_y);

				set_destination(firstTrackingWaypoint.x, firstTrackingWaypoint.y, target_alt, 0);
				
				first_tracking_waypoint++;
				ROS_INFO("Moving to first tracking point");
			}

			else if(check_waypoint_reached(pose_tolerance, heading_tolerance) == 1)
			{
				//Define trackingWaypoint as Coordinate structure to store nextWaypoint coordinates
				Coordinate trackingWaypoint;
				trackingWaypoint= getTrackingWaypoint(current_pos.x, current_pos.y, current_heading, dist2center_x, dist2center_y);

				set_destination(trackingWaypoint.x, trackingWaypoint.y, target_alt, 0);
				ROS_INFO("Moving to next tracking point");
			}

			ros::Rate tracking_rate(1.0);
			if (tracking_counter < tracking_time)
			{
				while(ros::ok())
				{	
					tracking_counter++;
					ROS_INFO("Tracking finished in %ld", tracking_time - tracking_counter);
					tracking_rate.sleep();
					break;
				}
			}

			else
			{
				ROS_INFO("Tracking done ! Back to searching !");
				// Publish to the topic of tracked vehicle coordinates
				tracked_vehicle_pos_pub.publish(position_msg);
    			//ROS_INFO("latitude is : %lf", position_msg.x);
				// Publish to the topic number of tracked vehicle 
				mode = 0;
			}
		}	
	}

return 0;
}
