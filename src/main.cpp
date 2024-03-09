#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <gnc_functions.hpp>

using namespace std;

//Image width and height
int width = 640;
int height = 480;
int img_center_x = width/2;
int img_center_y = height/2;

//Pixel to metres topic 
float pix2metres = 0.00332812; // Subscribes to pixel to metres topic // Need another callback function

//Distance to centres
float dist2center_x = 0;
float dist2center_y = 0;

// 0 = searching // 1 = tracking
int mode = 0;

//Flying altitude
float alt = 3;

// Time after which the drone goes back to searching if the target is lost during tracking 
int lost_target_time = 5;

//Tracking time
int tracking_time = 10;

//Target lost counter
int target_lost_counter = 0;

//Tracking counter
int tracking_counter = 0;

//Condition on first tracking waypoint
int first_tracking_waypoint = 0;

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) // callback function
{
	ROS_INFO("mode :  %ld", mode);

	for (int i=0; i<msg->bounding_boxes.size(); i++ ) // run on each bouding box that is in the message
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str()); //convert into a char array that can be used by ros; print detected object

		string boxe_name = msg->bounding_boxes[i].Class.c_str();

		if (boxe_name == "person") // condition should be changed to "rccars" We could also restrict the detection of Yolo to only rc cars and remove all the other classes to prevent issues
		{
			ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());
			mode = 1;
			target_lost_counter = 0; // initialise or put back target lost counter to 0

			//Extract the coordinates of the boxes
			int xMin = msg->bounding_boxes[i].xmin;
			int xMax = msg->bounding_boxes[i].xmax;
			int yMin = msg->bounding_boxes[i].ymin;
			int yMax = msg->bounding_boxes[i].ymax;

			//Print information
			ROS_INFO("Boxe number %ld", i);
			ROS_INFO("xmin: %ld", xMin);
			ROS_INFO("xmax: %ld", xMax);
			ROS_INFO("ymin: %ld", yMin);
			ROS_INFO("ymax: %ld", yMax);
			ROS_INFO("id: %ld", msg->bounding_boxes[i].id);

			//Get the center 
			float bb_center_x = xMin + (xMax - xMin) / 2;
			float bb_center_y = yMin + (yMax - yMin) / 2;
			ROS_INFO("Bounding Box center coordinates: (%lf, %lf)", bb_center_x, bb_center_y);

			//Compute distance to image center in pixel
			float pix_dist2center_x = bb_center_x - img_center_x;
			float pix_dist2center_y = bb_center_y - img_center_y;
			ROS_INFO("Center is %lf pixels away in x and %lf pixels away in y", pix_dist2center_x, pix_dist2center_y);

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

	ros::init(argc, argv, "detection_sub"); //name of the node
	ros::NodeHandle n; //enable connection to the ros network

	//	ros::Subscriber sub = <nodehandle>.subscribe("<topic>", <# of msg buffered>, <name of callback function>);
	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb); //1 = how many message buffered. default 1

	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// wait for FCU connection
	wait4connect();

	//Wait for user to set mode to guided
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(alt);

	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = alt;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 0;
	nextWayPoint.z = alt;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 5;
	nextWayPoint.z = alt;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 5;
	nextWayPoint.z = alt;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = alt;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);

	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0); // loop execution rate
	int counter = 0;
	while(ros::ok()) // loop as long as the node is running
	{	
		ROS_INFO("mode :  %ld", mode);

		if (mode == 0) //SEARCHING MODE
		{
			tracking_counter = 0;
			first_tracking_waypoint = 0;
			ros::spinOnce(); // Allows to continue processing main loop but still using callback information
			rate.sleep(); // Ensure ros rate 
			if(check_waypoint_reached(.3) == 1)
			{
				if (counter < waypointList.size())
				{
					set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
					counter++;	
				}
			}	
		}

		else if (mode == 1) //TRACKING MODE
		{
			ros::spinOnce();
			ROS_INFO("####TRACKING#####TRACKING#####TRACKING#########"); 
			geometry_msgs:: Point current_pos;
			current_pos = get_current_location();

			if(first_tracking_waypoint == 0)
			{
				ROS_INFO("Current position : %lf, %lf", current_pos.x, current_pos.y);
				set_destination(current_pos.x + dist2center_x, current_pos.y - dist2center_y, alt, 0);
				first_tracking_waypoint++;
				ROS_INFO("Moving to first tracking point");
			}else if(check_waypoint_reached(.3) == 1){
				set_destination(current_pos.x + dist2center_x, current_pos.y + dist2center_y, alt, 0);
				ROS_INFO("Moving to next tracking point");
			}

			ROS_INFO("tracking counter before the loop %ld", tracking_counter);
			ros::Rate tracking_rate(2.0);
			if (tracking_counter < tracking_time){
				while(ros::ok())
				{	ROS_INFO("tracking time %ld", tracking_time);
					ROS_INFO("tracking counter %ld", tracking_counter);
					tracking_counter++;
					ROS_INFO("Tracking finished in %ld", tracking_time - tracking_counter);
					tracking_rate.sleep();
					break;
				}
			}else{
				ROS_INFO("Tracking done ! Back to searching !");
				//Add coordinate of the vehicle ++ add condition on the tracking
				// Publish 
				mode = 0;
			}
		}	
	}

return 0;
}
