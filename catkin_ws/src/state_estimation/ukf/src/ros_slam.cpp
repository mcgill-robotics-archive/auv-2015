#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"

#include "ukf_slam.h"


ros::Publisher pub;
ros::Subscriber sub;
ukf_slam estimator;

Vector3d acc, gyro, slam;

Vector2d position; 

//Does this need to be changed to 2d?
void msgVectorToEigenVector(Ref<Vector3d> vector3d, geometry_msgs::Vector3 vector) {
    vector3d << vector.x, vector.y, vector.z;
}


void broadcastStaticFrames() {
  tf::TransformBroadcaster broadcaster;

  tf::Quaternion imuInternalHorizonToMountPoint;
  imuInternalHorizonToMountPoint.setRPY(PI/2,0,0);
  broadcaster.sendTransform(tf::StampedTransform(
    tf::Transform(imuInternalHorizonToMountPoint, zero), 
    ros::Time::now(), 
    "/IMU/initial_horizon", 
    "/IMU/mount"
  ));
  
  // TODO: Is there a quaternion constructor that combines these two lines
  tf::Quaternion imuMountToRobotFrame;
  imuMountToRobotFrame.setRPY(PI/2,0,0);

  tf::Transform imuMountToRobotTransform(imuMountToRobotFrame, tf::Vector3(0., 0., 0.));

  broadcaster.sendTransform(tf::StampedTransform(
    imuMountToRobotTransform, 
    ros::Time::now(), 
    "/IMU", 
    "/robot"
  ));
  broadcaster.sendTransform(tf::StampedTransform(
    imuMountToRobotTransform, 
    ros::Time::now(), 
    "/IMU/mount", 
    "/robot/initial_horizon"
  ));

  
  
}







void timerCallback(const ros::TimerEvent& event) {
  broadcastStaticFrames();
}



//Nicole: Copied directly from ros_pose but needs to be modified
//Already started modifying
void dataCallback(const sensor_msgs::Imu::ConstPtr& imu) {
    msgVectorToEigenVector(acc, imu->linear_acceleration);
    msgVectorToEigenVector(gyro, imu->angular_velocity);

    acc = -9.8 * acc.normalized(); //TODO(max) possible aliasing
    gyro *= 0.026;

    //Max: testing only
    //acc *= 0.;
    //gyro *= 0.;

    estimator.update(acc, gyro, slam); //pose changed to slam
    geometry_msgs::Quaternion quat = geometry_msgs::Quaternion();
    //printf("%10f\n", pose(0));
    poseToQuaternion(quat, pose);

//this whole section has to change 
    geometry_msgs::PoseStamped posStamped = geometry_msgs::PoseStamped();
    posStamped.header = imu->header;
    posStamped.header.frame_id = "base_footprint";
    posStamped.pose.orientation = quat;

    pub.publish(posStamped);
}



int main (int argc, char **argv) {
    ros::init(argc, argv, "slam_ukf");
    ros::NodeHandle node;


  tf::TransformBroadcaster broadcaster;

  ros::Timer timer = node.createTimer(ros::Duration(0.1), timerCallback);

  sub = node.subscribe("imu_data", 100, dataCallback);

return 0;
}
