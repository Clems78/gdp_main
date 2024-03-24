#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <gnc_functions.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <darknet_ros_msgs/ObjectCount.h>
#include <gdp_main/CoordinateList.h>

using namespace std;

/////////////////////////////////////////////// PARAMETERS ///////////////////////////////////////////////

//Flying altitude
float target_alt = 1.2;

//Check waypoints reached tolerance
float pose_tolerance = 0.5; //metres
float heading_tolerance = 7; //degrees

//Re tracking distance threshold
float tracking_coord_threshold = 0.001;

/////////////////////////////////////////////// VARIABLES ///////////////////////////////////////////////

bool tracking_flag;
bool undetected;

// Detection counter
int detection_counter_rt = 0;
int detection_counter = 0;
ros::Time last_zero_detection_time;

// 0 = searching // 1 = tracking
int mode = 0;

/////////////////////////////////////////////// PUBLISHER ///////////////////////////////////////////////

//tracked vehicle coordinate publisher 
ros::Publisher tracked_vehicle_pos_pub;

/////////////////////////////////////////////// MESSAGES ///////////////////////////////////////////////
//Initialise tracked vehicle coordinate message
geometry_msgs::Point position_msg;


/////////////////////////////////////////////// STRUCTURE ///////////////////////////////////////////////

//Coordinate structure
struct Coordinate 
{
    float x;
    float y;
};


/////////////////////////////////////////////// FUNCTIONS ///////////////////////////////////////////////

float calculateDistance(float trackedCoordLat, float trackedCoordLong, float droneLat, float droneLong)
{
    return sqrt(pow(droneLat - trackedCoordLat, 2) + pow(droneLong - trackedCoordLong, 2));
}

/////////////////////////////////////////////// CALLBACKS ///////////////////////////////////////////////

//const <msg package name>::<message>::ConstPtr& msg
void yolo_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& yolo_msg) // callback function
{
    // ROS_INFO("Yolo loop");
    // ROS_INFO("mode :  %ld", mode);
    // ROS_INFO("Tracking flag: %s", tracking_flag ? "true" : "false");

    if (tracking_flag == false)
    {
        for (int i=0; i<yolo_msg->bounding_boxes.size(); i++ ) // run on each bouding box that is in the message
        {
            // ROS_INFO("Number of bb detected: %d", yolo_msg->bounding_boxes.size()); //convert into a char array that can be used by ros; print detected object

            string boxe_name = yolo_msg->bounding_boxes[i].Class.c_str();

            if (boxe_name == "car" ) // 
            {
                undetected = true;
                ROS_INFO("Vehicule detected");
            }
        }
    }
}


//const <msg package name>::<message>::ConstPtr& msg
void object_count_cb(const darknet_ros_msgs::ObjectCount::ConstPtr& object_count_msg) // callback function
{
    detection_counter = object_count_msg->count;
    ROS_INFO("Object counter realtime : %d", detection_counter);
    ROS_INFO("Tracking flag: %s", tracking_flag ? "true" : "false");

    if (detection_counter != 0 && tracking_flag == true) {
        last_zero_detection_time = ros::Time::now();
    }

     ros::Time current_time = ros::Time::now();
     ros::Duration elapsed_time = current_time - last_zero_detection_time;
     ROS_INFO("Time elapsed since last_zero_detection_time: %f seconds", elapsed_time.toSec());

    if (tracking_flag == true && (ros::Time::now() - last_zero_detection_time).toSec() > 3.0)
    {
        tracking_flag = false;
        ROS_INFO("////////////////////Detection enabled again//////////////////:");
    }
}

//Get global coordinates of the drone
void pos_cb(const sensor_msgs::NavSatFix::ConstPtr& pos_msg)
{
    //Extract data
    float latitude = pos_msg->latitude;
    float longitude = pos_msg->longitude;
    float altitude = pos_msg->altitude;

    //Detail message content
    position_msg.x = latitude;
    position_msg.y = longitude;
    position_msg.z = altitude;
}

void already_tracked_cb(const gdp_main::CoordinateList::ConstPtr& already_tracked_msg)
{
    ROS_INFO("already tracked loop entered");
    ROS_INFO("Detection flag %ld", detection_counter);
    if (detection_counter != 1) // If target detect box detected 
    {
        //get current position in global coordinate
        float drone_lat = position_msg.x; 
        float drone_long = position_msg.y;

        //Boolean condition
        bool allDistanceAboveThreshold = true;

        for (int i=0; i<already_tracked_msg->points.size(); i++)
        {
            float already_tracked_x = already_tracked_msg->points[i].x;
            float already_tracked_y = already_tracked_msg->points[i].y;

            float distanceToCoord = calculateDistance(already_tracked_x, already_tracked_y, drone_lat, drone_long);

            if (distanceToCoord < tracking_coord_threshold)
            {
                allDistanceAboveThreshold = false;
                ROS_INFO("Target already tracked !");
            }
        }

        if (allDistanceAboveThreshold = true)
        {
            mode = 1; //Set to tracking 
            ROS_INFO("New target, commencing tracking !");
        }
    }
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "main"); //name of the node
    ros::NodeHandle n; //enable connection to the ros network

    //  ros::Subscriber sub = <nodehandle>.subscribe("<topic>", <# of msg buffered>, <name of callback function>);
    ros::Subscriber yolo_sub = n.subscribe("/drone2/bounding_boxes", 10, yolo_cb); //1 = how many message buffered. default 1

    ros::Subscriber object_count_sub = n.subscribe("/drone2/object_count", 1, object_count_cb); //1 = how many message buffered. default 1

    //Getting coordinate of already tracked vehicle 
    ros::Subscriber already_tracked_sub = n.subscribe("/all_tracked_vehicle_coords", 1, already_tracked_cb);

    //Getting global coordinate
    ros::Subscriber pos_sub = n.subscribe("/drone2/mavros/global_position/global", 1, pos_cb);

    // ros::Publisher pub = n.advertise<std_msgs::message-type>("created topic_name", 5);
    // Publish the position of the tracked vechile on another node
    tracked_vehicle_pos_pub = n.advertise<geometry_msgs::Point>("tracked_vehicle_pos", 1); //1000 is the queue size


    //initialize control publisher/subscribers
    init_publisher_subscriber(n);

    // wait for FCU connection
    // wait4connect();

    //create local reference frame 
    // initialize_local_frame();

    //specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
    ros::Rate rate(5.0); // loop execution rate

    while(ros::ok()) // loop as long as the node is running
    {   
        // ROS_INFO("MAIN LOOP");

        if (mode == 0) //SEARCHING MODE
        {   
            ROS_INFO("Searching");
        }

        else if (mode == 1) //TRACKING MODE
        {
            ROS_INFO("starting delay");
            ros::Duration delay(5.0);
            delay.sleep();
            mode = 0;
            tracking_flag = true;
            ROS_INFO("MODE TO 0");
        }   

        rate.sleep();
        ros::spinOnce();
    }
return 0;
}
