#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "auv_msgs/RangeBearingElevation.h"
#include "auv_msgs/SlamEstimate.h"
#include <string.h>
#include "ukf_slam.h"
#include <math.h>
#include <boost/lexical_cast.hpp>

ros::Subscriber sub;
ros::Publisher pub;
ukf_slam estimator(4);
tf::StampedTransform tf_sensor_transform;
Affine3d sensor_transform;

Vector3d measurement; //Holds the range bearing and elevation
Vector3d covariance; // Holds the covariance of the above

VectorXd position;


tf::TransformListener listener;

void dataCallback(const auv_msgs::RangeBearingElevation::ConstPtr& input) {
  static tf::TransformBroadcaster broadcaster;
  
  measurement << log(input->range), input->bearing, input->elevation;
  covariance << input->ln_range_variance, input->bearing_variance,
      input->elevation_variance;
  
  // TODO remove this and replace with a HashMap of names to IDs
  int objectID = atoi(input->name.c_str());
  
  // TODO make sure this works well if the transforms don't exist or are being published slowly
  // there are various exceptions we still need to catch
  // transform from "/north" frame to the sensor frame
  listener.lookupTransform("/north", input->header.frame_id,
      input->header.stamp, tf_sensor_transform);
      
  tf::transformTFToEigen(tf_sensor_transform, sensor_transform);
  
  position = estimator.update(objectID, sensor_transform, measurement, covariance); 
  
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
    
    auv_msgs::SlamEstimate estimate;
    estimate.ObjectID = i;
    estimate.xPos = position(2*i);
    estimate.yPos = position(2*i+1);
    MatrixXd covar = estimator.getCovariance(i);
    estimate.var_xx = covar(0,0);
    estimate.var_xy = covar(0,1);
    estimate.var_yy = covar(1,1);
    pub.publish(estimate);
  }

}


int main (int argc, char **argv) {
  ros::init(argc, argv, "slam_ukf");
  ros::NodeHandle node;
  sub = node.subscribe("slam/measurement", 100, dataCallback);
  pub = node.advertise<auv_msgs::SlamEstimate>("map_data", 100);
  ros::spin();
  return 0;
}

