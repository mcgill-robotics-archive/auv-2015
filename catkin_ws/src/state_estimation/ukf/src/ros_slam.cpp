#include "ros_slam.h"
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include "auv_msgs/SlamEstimate.h"
#include <string.h>
#include <math.h>
#include <boost/lexical_cast.hpp>
#include <boost/unordered_map.hpp>

ros_slam::ros_slam(ros::NodeHandle& node) :
  sub(node.subscribe("slam/measurement", 100, &ros_slam::dataCallback, this)),
  pub(node.advertise<auv_msgs::SlamEstimate>("map_data", 100)),
  estimator(4),
  currentIndex(2)
{
  auv_msgs::RangeBearingElevation rbe;
  ros::Publisher p2 = node.advertise<auv_msgs::RangeBearingElevation>("boo", 10);
  p2.publish(rbe);
}

void ros_slam::dataCallback(const auv_msgs::RangeBearingElevation::ConstPtr& input) {
  static tf::TransformBroadcaster broadcaster;
  
  measurement << log(input->range), input->bearing, input->elevation;
  covariance << input->ln_range_variance, input->bearing_variance,
      input->elevation_variance;
  
  if (map.find(input->name) == map.end())	//Does boost hashmaps work this way ? Might need an Iterator instead
  {
	  currentIndex = 3*currentIndex;	//Increment current index to accommodate for new object which is not already in hashmap
	  map[input->name] = currentIndex;	//Link new name to new index
  }
  else{	//Object already exists in hashmap
	  //Do nothing ?
  }
  
  int objectIndex = map.at(input->name);	//Declare returnIndex up there
  
  // TODO make sure this works well if the transforms don't exist or are being published slowly
  // there are various exceptions we still need to catch
  // transform from "/north" frame to the sensor frame
  // Also, we should switch to using the version that takes a fixed frame so
  // we can use odometry to compensate for sensor processing latency
  listener.lookupTransform("/north", input->header.frame_id,
      input->header.stamp, tf_sensor_transform);
      
  tf::transformTFToEigen(tf_sensor_transform, sensor_transform);
  
  position = estimator.update(objectIndex, sensor_transform, measurement, covariance); 
  
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

void callback(const auv_msgs::RangeBearingElevation::ConstPtr& s) {
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "slam_ukf");
  ros::NodeHandle node;
  (ros_slam(node));
  ros::spin();
  return 0;
}

