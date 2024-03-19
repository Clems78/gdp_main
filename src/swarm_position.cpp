#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <gdp_main/CoordinateList.h>
#include <gdp_main/DronePositionList.h>

using namespace std;

struct DronePosition 
{
    double latitude;
    double longitude;
    double altitude;
};

// Map to store positions of all drones with drone IDs as keys
map<int, DronePosition> drone_positions;


// Initialize publisher for drone positions
ros::Publisher drone_position_pub;


void publish_drone_positions()
{
    // Create a message to publish drone positions
    gdp_main::DronePositionList drone_position_msg;

    // Populate the message with drone positions
    for (const auto& entry : drone_positions) {
        drone_position_msg.latitude.push_back(entry.second.latitude);
        drone_position_msg.longitude.push_back(entry.second.longitude);
        drone_position_msg.altitude.push_back(entry.second.altitude);
    }

    // Publish the message
    drone_position_pub.publish(drone_position_msg);
}



void drone_position_cb(const sensor_msgs::NavSatFix::ConstPtr& drone_position_msg, int drone_id)
{
    // Store received position information into the corresponding element of the drone_positions map
    DronePosition position;
    position.latitude = drone_position_msg->latitude;
    position.longitude = drone_position_msg->longitude;
    position.altitude = drone_position_msg->altitude;
    drone_positions[drone_id] = position;

    // Print position information of all drones
    ROS_INFO("Drone %d position: Latitude: %f, Longitude: %f, Altitude: %f", drone_id + 1, position.latitude, position.longitude, position.altitude);
    
    // Publish the updated drone positions
    publish_drone_positions();

}


int main(int argc, char **argv) {

	ros::init(argc, argv, "swarm_position"); //name of the node
	ros::NodeHandle n; //enable connection to the ros network

	//Get number of drone from launch file 
	int drone_nb;
	drone_nb = 3;
	// n.param("drone_nb", drone_nb, 1); // default to one if not specified

	// Create an array of subscribers
   	ros::Subscriber drone_position_sub[drone_nb];

   	drone_position_pub = n.advertise<gdp_main::DronePositionList>("swarm_position", 10);

   	for (int i=0; i<drone_nb; i++)
	{
		//Create topic name // Should be changed because later on its going to be /droneX/main/tracked_vehicle_pos
		stringstream ss;
		ss << "/drone"<< i+1 << "/mavros/global_position/global";
		string topic_name = ss.str();
		// string topic_name = "/mavros/global_position/global";
		ROS_INFO("topic_name : %s", topic_name.c_str());
		drone_position_sub[i] = n.subscribe<sensor_msgs::NavSatFix>(topic_name, 1, boost::bind(drone_position_cb, _1, i));
	}

	ros::spin();

return 0;
}

