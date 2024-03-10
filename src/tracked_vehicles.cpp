#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;

//Subscribed to n number of topics where each main nodes publish the coordinate of the tracked vehicles 
//Each main node is also subscribed to this node so that if they don't track a vehicle that has already been tracked

// Set the threshold distance for taking into account a new tracked vehicle coordinate
float distance_threshold = 0.5; 

struct Coordinate 
{
	float latitude;
	float longitude;
	//float altitude;
};

vector<Coordinate> tracked_vehicle_coords_list;

// Function to calculate the distance between two coordinates
float calculateDistance(const Coordinate& coord1, const Coordinate& coord2) 
{
    float d_lat = coord2.latitude - coord1.latitude;
    float d_lon = coord2.longitude - coord1.longitude;
    return sqrt(d_lat * d_lat + d_lon * d_lon); // Euclidean distance (ignoring altitude)
}

// Function to check if a coordinate is within a threshold distance of any coordinate in the list
bool isWithinThreshold(const Coordinate& new_coord, float distance_threshold) 
{
    for (const auto& coord : tracked_vehicle_coords_list) 
    {
        if (calculateDistance(coord, new_coord) <= distance_threshold) 
        {
            return true; // Coordinate is within threshold distance of another coordinate in the list
        }
    }
    return false; // Coordinate is not within threshold distance of any coordinate in the list
}

void tracked_vehicle_cb(const geometry_msgs::Point::ConstPtr& tracked_vehicle_msg) // callback function
{
	float latitude = tracked_vehicle_msg->x;
	float longitude = tracked_vehicle_msg->y;
	//float altitude = tracked_vehicle_msg->z;

	ROS_INFO("latitude of the tracked vehicle : %lf", latitude);
	ROS_INFO("longitude of the tracked vehicle : %lf", longitude);

	Coordinate tracked_vehicle_coordinate;
	tracked_vehicle_coordinate.latitude = latitude;
	tracked_vehicle_coordinate.longitude = longitude;
	//tracked_vehicle_coordinate.altitude	= altitude;

	// Check if the new coordinate is within the threshold distance of any coordinate in the list
    if (!isWithinThreshold(tracked_vehicle_coordinate, distance_threshold)) 
    {
        // Add the new coordinate to the list if it's not within the threshold distance of any existing coordinate
        tracked_vehicle_coords_list.push_back(tracked_vehicle_coordinate);
        ROS_INFO("######Coordinates added to the tracked vehicle list !#######");
    }
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "tracked_vehicle"); //name of the node
	ros::NodeHandle nt; //enable connection to the ros network

	//Get number of drone from launch file 
	int drone_nb;
	nt.param("drone_nb", drone_nb, 1); // default to one if not specified

	//ros::Subscriber tracked_vehicle_pos_sub = nt.subscribe("/drone1/tracked_vehicle_pos", 1, tracked_vehicle_cb);

	// Create an array of subscribers
   	ros::Subscriber tracked_vehicle_pos_sub[drone_nb];

	// */
	for (int i=0; i<drone_nb; i++)
	{
		//Create topic name // Should be changed because later on its going to be /droneX/main/tracked_vehicle_pos
		stringstream ss;
		ss << "/drone"<< i+1 << "/tracked_vehicle_pos";
		string topic_name = ss.str();
		ROS_INFO("topic_name : %s", topic_name.c_str());
		tracked_vehicle_pos_sub[i] = nt.subscribe(topic_name, 1, tracked_vehicle_cb); //1 = how many message buffered. default 1
	}
	// */

	// Publish at 2Hz
	ros::Rate loop_rate(2);

	ros::spin(); //makes the int main run infinitely
	loop_rate.sleep();



return 0;
}
