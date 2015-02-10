#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>

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
	
	x1 = rand() % 1000 / 1000.0;
	x2 = rand() % 1000 / 1000.0;
	
	y = sqrt(-2*log(x1))*cos(2*M_PI*x2);
	
	return( mean + y * sigma);
}

geometry_msgs::Vector3 add_noise(geometry_msgs::Vector3 vector, float sigma) {
        geometry_msgs::Vector3 noisy_vector;
        noisy_vector.x = box_muller(vector.x, sigma);
        noisy_vector.y = box_muller(vector.y, sigma);
        noisy_vector.z = box_muller(vector.z, sigma);
        return noisy_vector;
}

int main(int argc, char **argv)
{
	srand(time(NULL));	

	ros::init(argc, argv, "sim_slam_input");

	ros::NodeHandle n;

	ros::Publisher velocity_pub = n.advertise<geometry_msgs::Vector3>("velocity", 1000);
	ros::Publisher relative_position_obj1_pub = n.advertise<geometry_msgs::Vector3>("relative_position_obj1", 1000);	
	ros::Publisher noisy_velocity_pub = n.advertise<geometry_msgs::Vector3>("noisy_velocity", 1000);
	ros::Publisher noisy_relative_position_obj1_pub = n.advertise<geometry_msgs::Vector3>("noisy_relative_position_obj1", 1000);
	
	//Update rate
	ros::Rate loop_rate(1000);
	
	//Define starting position of the robot
	geometry_msgs::Point robot_position;
	robot_position.x = 0; robot_position.y = 0; robot_position.z = 0;

	//Define velocity of the robot
	geometry_msgs::Vector3 robot_velocity;
	robot_velocity.x = 1000; robot_velocity.y = 0; robot_velocity.z = 0;
	
	//Define object locations
	geometry_msgs::Point obj1;
	obj1.x = 20000; obj1.y = 50000; obj1.z = 0;
	
	//Define noise
	float obj_noise = 1;
	float v_noise = 1;

	geometry_msgs::Vector3 relative_obj1_position;
	geometry_msgs::Vector3 noisy_relative_obj1_position;
	geometry_msgs::Vector3 noisy_robot_velocity;

	while (ros::ok())
	{
		relative_obj1_position = relative_position(robot_position, obj1);
		
		//Add noise to outputs
		noisy_relative_obj1_position = add_noise(relative_obj1_position, obj_noise);
		noisy_robot_velocity = add_noise(robot_velocity, v_noise);
		
		//Publish outputs
		velocity_pub.publish(robot_velocity);
		relative_position_obj1_pub.publish(relative_obj1_position);	
		noisy_velocity_pub.publish(noisy_robot_velocity);
		noisy_relative_position_obj1_pub.publish(noisy_relative_obj1_position);

		robot_position = update_position(robot_velocity, robot_position);
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	
	return 0;

}