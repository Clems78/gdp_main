#include <gnc_functions.hpp>
//include API 

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
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
	nextWayPoint.z = 1.2;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 2;
	nextWayPoint.y = 0;
	nextWayPoint.z = 1.2;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 2;
	nextWayPoint.y = 2;
	nextWayPoint.z = 1.2;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 2;
	nextWayPoint.z = 1.2;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		// ROS_INFO("outer loop");
		ros::spinOnce();
		rate.sleep();
		if(check_waypoint_reached(.3, 10) == 1)
		{
			ROS_INFO("Checkpoint fucking reached");
			if (counter < waypointList.size())
			{
				set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
				counter++;	
				ROS_INFO("Going to next waypoints");
			}else{
				//land after all waypoints are reached
				ROS_INFO("LANDING");
				land();
				break;
			}	
		}	
		
	}
	return 0;
}