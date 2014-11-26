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

void imuCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  ros::Time time(ros::Time::now());
  tf::TransformBroadcaster broadcaster;

  // orientation is the rotation from the initial_horizon frame of the imu to the imu
  // and so orientation = Rx(roll)*Ry(pitch)*Rz(yaw)*R(imu,robot)

  tf::Quaternion orientation = tf::Quaternion(
    msg->pose.orientation.x, 
    msg->pose.orientation.y, 
    msg->pose.orientation.z, 
    msg->pose.orientation.w
  );

  tf::Quaternion robot = imuMountToRobotFrame.inverse()*imuInternalHorizonToMountPoint.inverse()*orientation.inverse()*imuMountToRobotFrame;

  broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(robot, zero),
      time,
      "/initial_horizon",
      "/robot"
    )
  );

  double roll,pitch,yaw;
  
  tf::Matrix3x3(robot).getRPY(roll,pitch,yaw);

  tf::Quaternion yawQuat;
  yawQuat.setRPY(0,0,yaw);
  broadcaster.sendTransform(tf::StampedTransform(
    tf::Transform(yawQuat, zero),
    time,
    "/initial_horizon",
    "/horizon"
  ));
  

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;
  imuMountToRobotFrame.setRPY(PI/2,0,0);
  imuInternalHorizonToMountPoint.setRPY(PI/2,0,0);  

  // TODO: Figure out why nothing gets broadcast without this line
  tf::TransformBroadcaster broadcaster;
  // TODO: Do we need this on a timer?
  ros::Subscriber imuSub = node.subscribe("state_estimation/pose", 1000, imuCallBack);
  ros::spin();

  return 0;
}
