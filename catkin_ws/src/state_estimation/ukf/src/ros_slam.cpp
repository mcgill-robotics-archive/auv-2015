#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"

#include "ukf_slam.h"


ros::Subscriber sub;
ukf_slam estimator;

Vector2d measurement; //Holds the sonar measurement

Vector2d position; 





//Does this need to be changed to 2d? YES done.
void msgVectorToEigenVector(Ref<Vector2> vector2d, geometry_msgs::Vector3 vector) {
    vector3d << vector.x, vector.y;
}




//Nicole: Copied directly from ros_pose but needs to be modified
//Already started modifying
void dataCallback(const geometry_msgs::Vector3::ConstPtr& input) {
    msgVectorToEigenVector(meausurment, input);

    estimator.update(measurements, position); 

    broadcaster..sendTransform(
    tf::StampedTransform(
      tf::Transform(orientation, zero),
      ros::Time::now(),
      "/robot",
      "/gate"   
    )
  );

}


int main (int argc, char **argv) {
    ros::init(argc, argv, "slam_ukf");
    ros::NodeHandle node;

  tf::TransformBroadcaster broadcaster;

  sub = node.subscribe("noisy_relative_obj1_position", 100, dataCallback);

return 0;

}

