//Subscribe to magnetometer readings from IMU
//Check Ximupublisher in state_estimation for details of the magnetometer topic
//The node should create the tf transform between the tf 
//frame /initial_horizon and /north. Their definitions are on the wiki

#include <ros/ros.h>
#include <geometry_msgs/?.h> //Change this
int main(int argc, char **argv) {

    ros::init(argc, argv, "north_publisher");



    ros::NodeHandle n;



    ros::Subscriber sub = n.subscribe("Ximupublisher", 1000, callbackFunction); //What is actualy name of magnetometer ximupublisher



    ros::spin();

}



//callbackFunction gets called everytime the node youre subcsribed to publishes a message

//It suffices that the geometry_msgs::Pose part should be replaced with whatever the return type of the subscribed node is

void callbackFunction(const geometry_msgs::Pose::ConstPtr& msg) {

    ROS_INFO("%f", msg->pose.orientation.x) //change to ximu publisher
    //Here can use the data however we want
    //Will be a big function, maybe?

}

