#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.14159265358979323846

tf::Vector3 zero(0,0,0);

void broadcastStaticFrames() {
  tf::TransformBroadcaster broadcaster;
  /*broadcaster.sendTransform(
    tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), 
    tf::Vector3(0.297, 0.1435, 0.1225)), ros::Time::now(), "/robot", "/robot/rotation_center")
  );*/


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

void imuCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  ros::Time time(ros::Time::now());
  tf::TransformBroadcaster broadcaster;

  tf::Quaternion orientation = tf::Quaternion(
    msg->pose.orientation.x, 
    msg->pose.orientation.y, 
    msg->pose.orientation.z, 
    msg->pose.orientation.w
  );


  // orientation is the rotation from the initial_horizon frame of the imu to the imu
  // and so orientation = Rx(roll)*Ry(pitch)*Rz(yaw)*R(imu,robot)

  broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(orientation, zero),
      time,
      "/IMU",
      "/IMU/initial_horizon"   
    )
  );

  /*
  * This doesn't work because of tf and threading
  tf::Transformer transformer;
  if (transformer.canTransform("/robot/initial_horizon", "/robot", time)) {
  tf::StampedTransform robotTransform;
  double roll,pitch,yaw;
  
  transformer.lookupTransform("/robot/initial_horizon", "/robot", time, robotTransform);
  robotTransform.getBasis().getRPY(roll,pitch,yaw);

  tf::Quaternion yawQuat;
  yawQuat.setRPY(0,0,yaw);
  broadcaster.sendTransform(tf::StampedTransform(
    tf::Transform(yawQuat, zero),
    time,
    "/robot/initial_horizon",
    "/robot/horizon"
  ));*/
  

}

void timerCallback(const ros::TimerEvent& event) {
  broadcastStaticFrames();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;  

  // TODO: Figure out why nothing gets broadcast without this line
  tf::TransformBroadcaster broadcaster;
  // TODO: Do we need this on a timer?
  ros::Timer timer = node.createTimer(ros::Duration(0.1), timerCallback);
  ros::Subscriber imuSub = node.subscribe("state_estimation/pose", 1000, imuCallBack);
  ros::spin();

  return 0;
}
