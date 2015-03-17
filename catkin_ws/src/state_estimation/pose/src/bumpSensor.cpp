#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"

int changeThreshold = 1;
int lastx = 0;
int lasty=0;
int lastz=0;
int currentx, currenty, currentz;
void chatterCallback(const geometry_msgs::Vector3 msg)
{
  //ROS_INFO("I heard: [%f]", msg.x);
  currentx = msg.x;
  currenty=msg.y;
  currentz=msg.z;
  if ( ((currentx*currentx + currenty*currenty + currentz*currentz) - (lastx*lastx + lasty*lasty + lastz*lastz))>1) {
	ROS_INFO("BUMP!");
  }	
  lastx = currentx;
  lasty=currenty;
  lastz=currentz;
}

int main(int argc, char **argv){
ros::init(argc, argv, "bumpSensor");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("/state_estimation/acc", 1000, chatterCallback);
ros::spin();
return 0;
}
//http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
//change CMakeList files to include this node in the catkin build!
