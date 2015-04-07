#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>
#include "auv_msgs/SlamTarget.h"
#include "auv_msgs/SlamEstimate.h"
#include <string.h>
#include "ukf_slam.h"
#include <boost/lexical_cast.hpp>

ros::Subscriber sub;
ros::Publisher pub;
ukf_slam estimator(4);

Vector2d measurement; //Holds the sonar measurement

VectorXd position(8); 

//Does this need to be changed to 2d? YES done.
//void msgVectorToEigenVector(Ref<Vector2d> vector2d, 
//    const geometry_msgs::Vector3::ConstPtr& vector) {
//  vector2d << vector->x, vector->y;
//}

void dataCallback(const auv_msgs::SlamTarget::ConstPtr& input) {
  static tf::TransformBroadcaster broadcaster;
  
  measurement << input->xPos, input->yPos;
  int objectID =  input->ObjectID;
  
  estimator.update(measurement, position, objectID); 
  for(int i = 0; i < 4; i++) {
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion::getIdentity(),
            tf::Vector3(position(2*i), position(2*i + 1), 0.)),
        ros::Time::now(),
        "robot",
        boost::lexical_cast<std::string>(i)    
      )
    );
  }
  
  auv_msgs::SlamEstimate estimate;
  estimate.ObjectID = objectID;
  estimate.xPos = position(2*objectID);
  estimate.yPos = position(2*objectID+1);
  MatrixXd covar = estimator.getCovariance(objectID);
  estimate.var_xx = covar(0,0);
  estimate.var_xy = covar(0,1);
  estimate.var_yy = covar(1,1);
  pub.publish(estimate);

}


int main (int argc, char **argv) {
  ros::init(argc, argv, "slam_ukf");
  ros::NodeHandle node;
  //tf::TransformBroadcaster broadcaster;
  sub = node.subscribe("sim_slam/position/noisy", 100, dataCallback);
  pub = node.advertise<auv_msgs::SlamEstimate>("map_data", 100);
  ros::spin();
  return 0;
}

