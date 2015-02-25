#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "auv_msgs/SlamTarget.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

boost::mt19937 rng; // I don't seed it on purpouse (it's not relevant)
boost::normal_distribution<> nd(0.0, 1.0);
boost::variate_generator<boost::mt19937&, 
    boost::normal_distribution<> > var_nor(rng, nd);


geometry_msgs::Point update_position(geometry_msgs::Vector3 velocity, geometry_msgs::Point previous_position) {
	geometry_msgs::Point new_position;
	new_position.x = previous_position.x + velocity.x;
	new_position.y = previous_position.y + velocity.y;
	new_position.z = previous_position.z + velocity.z;
	return new_position;
}

geometry_msgs::Vector3 relative_position(geometry_msgs::Point robot_position, geometry_msgs::Point object_position) {
	geometry_msgs::Vector3 relative_position;
	relative_position.x = object_position.x - robot_position.x;
	relative_position.y = object_position.y - robot_position.y;
	relative_position.z = object_position.z - robot_position.z;
	return relative_position;
}

float box_muller(float mean, float sigma) {
	float x1, x2, y;
	
	x1 = rand()/ 1000.0;
	x2 = rand() % 1000 / 1000.0;
	if (x1 == 0) x1 = 0.5;
	y = sqrt(-2*log(x1))*cos(2*M_PI*x2);
	return( mean + y * sigma);
}

geometry_msgs::Vector3 add_noise(geometry_msgs::Vector3 vector, float sigma) {
        geometry_msgs::Vector3 noisy_vector;
        noisy_vector.x = vector.x + var_nor() * sigma;
        noisy_vector.y = vector.y + var_nor() * sigma;
        noisy_vector.z = 0.;;
        return noisy_vector;
}

int main(int argc, char **argv)
{
	srand(time(NULL));	

	ros::init(argc, argv, "sim_slam_input");

	ros::NodeHandle n;
	tf::TransformBroadcaster broadcaster;

	ros::Publisher velocity_pub = n.advertise<geometry_msgs::Vector3>("velocity", 1000);
	ros::Publisher position_pub = n.advertise<auv_msgs::SlamTarget>("sim_slam/position/actual", 1000);	
	ros::Publisher noisy_velocity_pub = n.advertise<geometry_msgs::Vector3>("noisy_velocity", 1000);
	ros::Publisher noisy_position_pub = n.advertise<auv_msgs::SlamTarget>("sim_slam/position/noisy", 1000);
	
	//Update rate
	ros::Rate loop_rate(10);
	
	//Define starting position of the robot
	geometry_msgs::Point robot_position;
	robot_position.x = 0; robot_position.y = 0; robot_position.z = 0;

	//Define velocity of the robot
	geometry_msgs::Vector3 robot_velocity;
	robot_velocity.x = 0.0; robot_velocity.y = 0; robot_velocity.z = 0;
	
	//Define object locations
	const int numObjs = 4;
	geometry_msgs::Point objs[4];
  objs[0].x = 3; objs[0].y = 0; objs[0].z = 0;
  objs[1].x = 0; objs[1].y = 3; objs[1].z = 0;
  objs[2].x = -3; objs[2].y = 0; objs[2].z = 0;
  objs[3].x = 0; objs[3].y = -3; objs[3].z = 0;
    	
	//Define noise
	float obj_noise = 1;
	float v_noise = 1;

	geometry_msgs::Vector3 relative_obj1_position;
	geometry_msgs::Vector3 noisy_relative_position;
	geometry_msgs::Vector3 noisy_robot_velocity;

	while (ros::ok())
	{
		
		//Add noise to outputs
		noisy_robot_velocity = add_noise(robot_velocity, v_noise);
		
		//Publish outputs
		velocity_pub.publish(robot_velocity);
		noisy_velocity_pub.publish(noisy_robot_velocity);
		
		for (int i = 0; i < numObjs; i++) {
		  auv_msgs::SlamTarget noisy_msg, actual_msg;
		  noisy_msg.ObjectID = i;
		  actual_msg.ObjectID = i;
		  noisy_relative_position = add_noise(relative_position(robot_position, objs[i]),obj_noise);
		  noisy_msg.xPos = noisy_relative_position.x;
		  noisy_msg.yPos = noisy_relative_position.y;
		  actual_msg.xPos = relative_position(robot_position, objs[i]).x;
		  actual_msg.yPos = relative_position(robot_position, objs[i]).y;
		  noisy_position_pub.publish(noisy_msg);
		  position_pub.publish(actual_msg);
		}

		robot_position = update_position(robot_velocity, robot_position);
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	
	return 0;

}
