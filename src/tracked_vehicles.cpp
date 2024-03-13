#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <gdp_main/CoordinateList.h>
#include <vector>

using namespace std;

//Subscribed to n number of topics where each main nodes publish the coordinate of the tracked vehicles 
//Each main node is also subscribed to this node so that if they don't track a vehicle that has already been tracked

// Set the threshold distance for taking into account a new tracked vehicle coordinate
float distance_threshold = 0.000000000000000001; 

struct Coordinate 
{
	float x;
	float y;
	//float altitude;
};

Coordinate tracked_vehicle_coordinate;

vector<Coordinate> tracked_vehicle_coords_list;

//Initilise publisher 
ros::Publisher already_tracked_vehicle_pub;



// Function to calculate the distance between two coordinates
float calculateDistance(const Coordinate& coord1, const Coordinate& coord2) 
{
    float d_lat = coord2.x - coord1.x;
    float d_lon = coord2.y - coord1.y;
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
	float altitude = tracked_vehicle_msg->z;

	ROS_INFO("latitude of the tracked vehicle : %lf", latitude);
	ROS_INFO("longitude of the tracked vehicle : %lf", longitude);

	tracked_vehicle_coordinate.x = latitude;
	tracked_vehicle_coordinate.y = longitude;
	//tracked_vehicle_coordinate.altitude	= altitude;

	// Check if the new coordinate is within the threshold distance of any coordinate in the list
    if (!isWithinThreshold(tracked_vehicle_coordinate, distance_threshold)) 
    {
        // Add the new coordinate to the list if it's not within the threshold distance of any existing coordinate
        tracked_vehicle_coords_list.push_back(tracked_vehicle_coordinate);
        ROS_INFO("######Coordinates added to the tracked vehicle list !#######");

    }

	//Detail message content
	//already_tracked_vehicle_msg.x = latitude;
    //already_tracked_vehicle_msg.y = longitude;
    //already_tracked_vehicle_msg.z = 0;
    //already_tracked_vehicle_pub.publish(already_tracked_vehicle_msg);
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "tracked_vehicle"); //name of the node
	ros::NodeHandle n; //enable connection to the ros network

	//Get number of drone from launch file 
	int drone_nb;
	n.param("drone_nb", drone_nb, 1); // default to one if not specified

	// Create an array of subscribers
   	ros::Subscriber tracked_vehicle_pos_sub[drone_nb];

	for (int i=0; i<drone_nb; i++)
	{
		//Create topic name // Should be changed because later on its going to be /droneX/main/tracked_vehicle_pos
		stringstream ss;
		ss << "/drone"<< i+1 << "/tracked_vehicle_pos";
		string topic_name = ss.str();
		ROS_INFO("topic_name : %s", topic_name.c_str());
		tracked_vehicle_pos_sub[i] = n.subscribe(topic_name, 1, tracked_vehicle_cb); //1 = how many message buffered. default 1
	}
	//Initialise tracked vehicle coordinate message
	gdp_main::CoordinateList already_tracked_vehicle_msg;

	already_tracked_vehicle_pub = n.advertise<gdp_main::CoordinateList>("all_tracked_vehicle_coords", 1);


	// Publish at 2Hz
	ros::Rate rate(2);

	 //makes the int main run infinitely

	while(ros::ok())
	{
		already_tracked_vehicle_msg.points.clear();

	    for (vector<Coordinate>::iterator it = tracked_vehicle_coords_list.begin(); it != tracked_vehicle_coords_list.end(); ++it) 
	    {
	    	geometry_msgs::Point tracked_vehicle_coordinate;

	        //geometry_msgs::Point point;
	        tracked_vehicle_coordinate.x = (*it).x;
	        tracked_vehicle_coordinate.y = (*it).y;
	        tracked_vehicle_coordinate.z = 0;
	        already_tracked_vehicle_msg.points.push_back(tracked_vehicle_coordinate);
	    }

	    already_tracked_vehicle_pub.publish(already_tracked_vehicle_msg);
	    
	    ros::spinOnce();
	    rate.sleep();




	}


return 0;
}
