#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

using namespace std;

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{

	for (int i=0; i<msg->bounding_boxes.size(); i++ ) // run on each bouding box that is in the message
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str()); //convert into a char array that can be used by ros
	}
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "detection_sub"); //name of the node
	ros::NodeHandle n; //enable connection to the ros network

	//	ros::Subscriber sub = <nodehandle>.subscribe("<topic>", <# of msg buffered>, <name of callback function>);
	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb); //1 = how many message buffered. default 1


	ros::spin(); //makes the int main run infinitely


return 0;
}

