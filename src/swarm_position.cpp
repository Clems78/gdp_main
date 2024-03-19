#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Point.h>

using namespace std;


void drone_position_cb(const geometry_msgs::Point::ConstPtr& drone_position_msg)
{

}


int main(int argc, char **argv) {

	ros::init(argc, argv, "swarm_position"); //name of the node
	ros::NodeHandle n; //enable connection to the ros network

	//Get number of drone from launch file 
	int drone_nb;
	n.param("drone_nb", drone_nb, 1); // default to one if not specified

	// Create an array of subscribers
   	ros::Subscriber drone_position_sub[drone_nb];

   	for (int i=0; i<drone_nb; i++)
	{
		//Create topic name // Should be changed because later on its going to be /droneX/main/tracked_vehicle_pos
		stringstream ss;
		ss << "/drone"<< i+1 << "/drone_position";
		string topic_name = ss.str();
		ROS_INFO("topic_name : %s", topic_name.c_str());
		drone_position_sub[i] = n.subscribe(topic_name, 1, drone_position_cb); //1 = how many message buffered. default 1
	}





return 0;
}

