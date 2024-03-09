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
	set_mode("GUIDED");

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(1.2);

	geometry_msgs:: Point current_pos;
	current_pos = get_current_location();
	ROS_INFO("Current position: %lf, %lf", current_pos.x, current_pos.y);

	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = -35.363260;
	nextWayPoint.y = 149.165171;
	nextWayPoint.z = 1.2;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = -35.363260;
	nextWayPoint.y = 149.165158;
	nextWayPoint.z = 1.2;
	nextWayPoint.psi = 0;


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		if(check_waypoint_reached(.3) == 1)
		{
				set_destination(-35, 149, 1.2, 0);
				counter++;	
		}else{
			//land after all waypoints are reached
			land();
			}			
	}
	return 0;
}