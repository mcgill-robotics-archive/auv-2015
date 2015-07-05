#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.14159265358979323846

tf::Vector3 zero(0,0,0);

tf::Quaternion imuMountToRobotFrame;
tf::Quaternion imuInternalHorizonToMountPoint;
tf::Quaternion initialHorizonToRobot;

void imuCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  ros::Time time(ros::Time::now());
  tf::TransformBroadcaster broadcaster;

  tf::Quaternion orientation(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w
  );

  initialHorizonToRobot = imuMountToRobotFrame.inverse() * imuInternalHorizonToMountPoint.inverse() *
                          orientation.inverse()*imuMountToRobotFrame;

  broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(initialHorizonToRobot, zero),
      time,
      "/initial_horizon",
      "/robot"
    )
  );
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;
  imuMountToRobotFrame.setRPY(0,0,PI/2);
  imuInternalHorizonToMountPoint.setRPY(PI,0,0);

  // TODO: Figure out why nothing gets broadcast without this line
  tf::TransformBroadcaster broadcaster;
  ros::Subscriber imuSub = node.subscribe("state_estimation/pose", 1000, imuCallBack);
  ros::spin();

  return 0;
}
