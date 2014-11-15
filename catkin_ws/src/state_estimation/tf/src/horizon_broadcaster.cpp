#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/* Quick and dirty way of broadcasting horizon frame for
 * dry test
 */

tf::Vector3 zero(0.,0.,0.);

void broadcastHorizonFrame() {
  tf::TransformBroadcaster broadcaster;
  tf::TransformListener listener;

  ros::Time t(ros::Time::now());
  listener.waitForTransform("/robot/initial_horizon", "/robot", t, ros::Duration(0.3));
  if (listener.canTransform("/robot/initial_horizon", "/robot", ros::Time(0))) {
    tf::StampedTransform robotTransform;
    listener.lookupTransform("/robot/initial_horizon", "/robot", ros::Time(0), robotTransform);
    
    double roll,pitch,yaw;
    robotTransform.getBasis().getRPY(roll,pitch,yaw);

    tf::Quaternion yawQuat;
    yawQuat.setRPY(0,0,yaw);
    broadcaster.sendTransform(tf::StampedTransform(
      tf::Transform(yawQuat, zero),
      robotTransform.stamp_,
      "/robot/initial_horizon",
      "/robot/horizon"
    ));
  }
}

void timerCallback(const ros::TimerEvent& event) {
  broadcastHorizonFrame();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "horizon_broadcaster");
  ros::NodeHandle node;  

  // TODO: Figure out why nothing gets broadcast without this line
  tf::TransformBroadcaster broadcaster;
    
  ros::Timer timer = node.createTimer(ros::Duration(0.03), timerCallback);
  ros::spin();

  return 0;
}
