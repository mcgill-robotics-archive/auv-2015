#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

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

geometry_msgs::Vector3 add_noise(geometry_msgs::Vector3 vector, float noise) {
	geometry_msgs::Vector3 noisy_vector;
	srand(time(NULL) ^ getpid());
	noisy_vector.x = vector.x + noise/2 - noise*(rand()%1000 / 1000.0);
	noisy_vector.y = vector.y + noise/2 - noise*(rand()%1000 / 1000.0);
	noisy_vector.z = vector.z + noise/2 - noise*(rand()%1000 / 1000.0);
	return noisy_vector;
}

int main(int argc, char **argv)
{
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
	float obj_noise = 10;
	float v_noise = 100;

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
		
		//ROS_INFO("Robot Position: \n x: %f \n y: %f \n z: %f", robot_position.x, robot_position.y, robot_position.z);
		//ROS_INFO("Robot Velocity: \n x: %f \n y: %f \n z: %f", robot_velocity.x, robot_velocity.y, robot_velocity.z);
		//ROS_INFO("Noisy Robot Velocity: \n x: %f \n y: %f \n z: %f", noisy_robot_velocity.x, noisy_robot_velocity.y, noisy_robot_velocity.z);		

		ros::spinOnce();
		
		loop_rate.sleep();
	}
	
	return 0;

}
