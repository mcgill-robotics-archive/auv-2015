#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"

#include "ukf_pose.h"

ros::Publisher pub;
ros::Subscriber sub;
ukf_pose estimator;
Vector3d acc, gyro, pose;
Vector3d quaternion;

void msgVectorToEigenVector(Ref<Vector3d> vector3d, geometry_msgs::Vector3 vector) {
	vector3d << vector.x, vector.y, vector.z;
}

void eigenToMsgQuaternion(geometry_msgs::Quaternion& msgQ, Quaterniond eigenQ) {
	msgQ.w = eigenQ.w();
	msgQ.x = eigenQ.x();
	msgQ.y = eigenQ.y();
	msgQ.z = eigenQ.z();
}

void dataCallback(const sensor_msgs::Imu::ConstPtr& imu) {
	msgVectorToEigenVector(acc, imu->linear_acceleration);
	msgVectorToEigenVector(gyro, imu->angular_velocity);

	acc = -9.8 * acc.normalized(); //TODO(max) possible aliasing
	gyro *= 0.026;


	estimator.update(acc, gyro, pose);
	geometry_msgs::Quaternion quat = geometry_msgs::Quaternion();
	eigenToMsgQuaternion(quat, Quaterniond(AngleAxisd(pose.norm(), pose.normalized())));

	geometry_msgs::PoseStamped posStamped = geometry_msgs::PoseStamped();
	posStamped.header = imu->header;
	posStamped.header.frame_id = "base_footprint";
	posStamped.pose.orientation = quat;

	pub.publish(posStamped);
}


int main (int argc, char **argv) {
	ros::init(argc, argv, "pose_ukf");
	ros::NodeHandle node;

	pub = node.advertise<geometry_msgs::PoseStamped>("ukf", 100);
	sub = node.subscribe("imu_data", 100, dataCallback);

	ros::spin();

	return 0;
}
