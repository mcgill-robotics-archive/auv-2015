#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "ukf_velocity.h"

ukf_velocity estimator;
Vector2d velocity;
ros::Publisher pub;
ros::Subscriber sub;

void imuCallback(const geometry_msgs::Vector3::ConstPtr& acc) {
	Vector2d XYacc; 
	XYacc(0) = acc->x; 
	XYacc(1) = acc->y;
	
	estimator.update(XYacc, 0.1, velocity);
	geometry_msgs::Vector3 publishVelocity;
	publishVelocity.x = velocity(0);
	publishVelocity.y = velocity(1);
	pub.publish(publishVelocity);
}
	
int main(int argc, char **argv){
	ros::init(argc, argv, "velocity_ukf");
	ros::NodeHandle node;
	ros::Rate r(10);
		
	pub = node.advertise<geometry_msgs::Vector3>("ukf_velocity", 100);
	sub = node.subscribe("state_estimation/acc", 100, imuCallback);
	ros::spin();
	r.sleep();

	return 0;
}	
