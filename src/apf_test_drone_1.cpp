#include <gnc_functions.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>

sensor_msgs::NavSatFix drone1_position;
sensor_msgs::NavSatFix drone2_position;

float drone1_x, drone1_y, drone2_x, drone2_y;
double repulsive_force, attractive_force, total_force, theta;
double k_att = 0.5, k_rep = 0.5, eta = 0.5;
double min_distance = 2.0;

void drone1_global_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    drone1_position = *msg;
    drone1_x = drone1_position.latitude;
    drone1_y = drone1_position.longitude;
}

void drone2_global_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    drone2_position = *msg;
    drone2_x = drone2_position.latitude;
    drone2_y = drone2_position.longitude;
}

void calculate_apf(double lat_wp, double lon_wp, float& new_x, float& new_y)
{
    // Convert waypoint latitude and longitude to local Cartesian coordinates (x_wp, y_wp) using a simple approximation
    double x_wp = (lon_wp - drone1_y) * M_PI * 40000.0 / 180.0;
    double y_wp = (lat_wp - drone1_x) * M_PI * 40000.0 / 180.0;

    double dx = x_wp;
    double dy = y_wp;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance < min_distance)
    {
        repulsive_force = k_rep * (1 / distance - 1 / min_distance);
        attractive_force = k_att * sqrt(dx * dx + dy * dy);
        total_force = repulsive_force + attractive_force;
        theta = atan2(dy, dx);

        new_x = drone1_x + eta * total_force * cos(theta);
        new_y = drone1_y + eta * total_force * sin(theta);
    }
    else
    {
        new_x = x_wp;
        new_y = y_wp;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");

    ros::Subscriber drone1_global_position_sub = gnc_node.subscribe<sensor_msgs::NavSatFix>("drone1/mavros/global_position/global", 10, drone1_global_position_callback);
    ros::Subscriber drone2_global_position_sub = gnc_node.subscribe<sensor_msgs::NavSatFix>("drone2/mavros/global_position/global", 10, drone2_global_position_callback);

    init_publisher_subscriber(gnc_node);

    wait4connect();

    set_mode("GUIDED");

    initialize_local_frame();

    takeoff(5);

  //specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 1.2;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 5;
	nextWayPoint.z = 1.2;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);

    ros::Rate rate(2.0);
    int counter = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();

        float new_x, new_y;
        // calculate_apf(waypointList[counter].x, waypointList[counter].y, new_x, new_y);

        if(check_waypoint_reached(.3, 10) == 1)
        {
            if (counter < waypointList.size())
            {
                // set_destination(new_x, new_y, waypointList[counter].z, waypointList[counter].psi);
                set_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);
                counter++;
            }
            else
            {
                land();
                break;
            }
        }
    }

    return 0;
}
