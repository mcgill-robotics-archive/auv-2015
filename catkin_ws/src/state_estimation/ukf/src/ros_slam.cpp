#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>
#include "auv_msgs/RangeBearingElevation.h"
#include "auv_msgs/SlamEstimate.h"
#include <string.h>
#include "ukf_slam.h"
#include <boost/lexical_cast.hpp>
#include <boost/unordered_map.hpp>

ros::Subscriber sub;
ros::Publisher pub;
ukf_slam estimator(4);
boost::unordered_map<std::string,int> map;
int currentIndex = 2;	//Assuming first 3 are robot's state ?;

Vector3d measurement; //Holds the range bearing and elevation
Vector3d covariance; // Holds the covariance of the above

VectorXd position; 

//Does this need to be changed to 2d? YES done.
//void msgVectorToEigenVector(Ref<Vector2d> vector2d, 
//    const geometry_msgs::Vector3::ConstPtr& vector) {
//  vector2d << vector->x, vector->y;
//}

tf::TransformListener listener;

void dataCallback(const auv_msgs::RangeBearingElevation::ConstPtr& input) {
  static tf::TransformBroadcaster broadcaster;
  
  measurement << input->ln_range, input->bearing, input->elevation;
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
  // TODO remove this and replace with a HashMap of names to IDs -> Done
  int returnIndex = map.at(input->name);	//Declare returnIndex up there
  
  // TODO make sure this works well if the transforms don't exist or are being published slowly
  // there are various exceptions we still need to catch
  // transform from "/north" frame to the sensor frame
  tf::Transform transform(listener.lookupTransform(input->header.frame, "/north", input->header.time));
  
  output = estimator.update(returnIndex, transform, measurement, covariance); 
  
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

