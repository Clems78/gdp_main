#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/State.h>

void drone_status_cb(const mavros_msgs::State::ConstPtr& state_msg)
{
    if (!state_msg->armed) {
        ROS_INFO("Task Manager: Vehicle Disarmed");
    } else {
        ROS_INFO("Task Manager: Vehicle Armed");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "swarm_position"); // Initialize ROS node
    ros::NodeHandle n; // Create a handle for this node

    // Get number of drones from launch file 
    int drone_nb;
    n.param("drone_nb", drone_nb, 1); // Default to one if not specified

    // Create an array of subscribers
    ros::Subscriber drone_status_sub[drone_nb];

    for (int i = 0; i < drone_nb; i++) {
        // Create topic name
        std::stringstream ss;
        //ss << "/drone" << i + 1 << "/mavros/state";
        ss << "/mavros/state";
        std::string topic_name = ss.str();
        ROS_INFO("topic_name: %s", topic_name.c_str());
        drone_status_sub[i] = n.subscribe<mavros_msgs::State>(topic_name, 1, drone_status_cb); // 1 = how many message buffered. Default 1
    }

    ros::spin(); // Enter ROS event loop

    return 0;
}