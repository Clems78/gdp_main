#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <tuple>

using namespace std;

//Image width and height
int width = 640;
int height = 480;
int img_center_x = width/2;
int img_center_y = height/2;

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) // callback function
{

	for (int i=0; i<msg->bounding_boxes.size(); i++ ) // run on each bouding box that is in the message
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str()); //convert into a char array that can be used by ros; print detected object

		string boxe_name = msg->bounding_boxes[i].Class.c_str();

		if (boxe_name == "person") // condition should be changed to "rccars" We could also restrict the detection of Yolo to only rc cars and remove all the other classes to prevent issues
		{
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

			//Compute distance to image center
			float dist2center_x = bb_center_x - img_center_x;
			float dist2center_y = bb_center_y - img_center_y;
			ROS_INFO("Center is %lf pixels away in x and %lf pixels away in y", dist2center_x, dist2center_y);
		}
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

