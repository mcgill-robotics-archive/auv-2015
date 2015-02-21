#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>

#include "ukf_slam.h"

ros::Subscriber sub;
ukf_slam estimator;

Vector2d measurement; //Holds the sonar measurement

Vector2d position; 

//Does this need to be changed to 2d? YES done.
void msgVectorToEigenVector(Ref<Vector2d> vector2d, 
    const geometry_msgs::Vector3::ConstPtr& vector) {
  vector2d << vector->x, vector->y;
}

void dataCallback(const geometry_msgs::Vector3::ConstPtr& input) {
  static tf::TransformBroadcaster broadcaster;
  
  msgVectorToEigenVector(measurement, input);
  estimator.update(measurement, position); 
  broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(tf::Quaternion::getIdentity(),
          tf::Vector3(position(0), position(1), 0.)),
      ros::Time::now(),
      "/robot",
      "/gate"   
    )
  );

}


int main (int argc, char **argv) {
  ros::init(argc, argv, "slam_ukf");
  ros::NodeHandle node;
  //tf::TransformBroadcaster broadcaster;
  sub = node.subscribe("noisy_relative_obj1_position", 100, dataCallback);
  return 0;
}

