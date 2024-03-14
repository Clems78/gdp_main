#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/Point.h>
#include <gnc_functions.hpp>

using namespace std;

int mode = 0;

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) // callback function
{
	for (int i=0; i<msg->bounding_boxes.size(); i++ ) // run on each bouding box that is in the message
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str()); //convert into a char array that can be used by ros; print detected object

		string boxe_name = msg->bounding_boxes[i].Class.c_str();

		if (boxe_name == "truck") // condition should be changed to "rccars" We could also restrict the detection of Yolo to only rc cars and remove all the other classes to prevent issues
		{
			ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());
			mode = 1;
		}
	}
}


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
	takeoff(1.2);


	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 5;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0); // loop execution rate
	int counter = 0;
	while(ros::ok()) // loop as long as the node is running
	{
		if (mode == 0)
		{

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

		if (mode == 1)
		{
			ROS_INFO("LANDING");
			land();
			break; // finish the program
		}

		
	}


return 0;
}
