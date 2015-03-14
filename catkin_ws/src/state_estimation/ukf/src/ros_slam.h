#ifndef ROS_SLAM_H_
#define ROS_SLAM_H_

#include "ros/ros.h"
#include "ukf_slam.h"
#include "auv_msgs/RangeBearingElevation.h"
#include <tf/transform_listener.h>

class ros_slam
{
  public:
    ros_slam(ros::NodeHandle&);
            
  private:
    void dataCallback(const auv_msgs::RangeBearingElevation::ConstPtr& input);
    ros::Publisher pub;
    ros::Subscriber sub;
    ukf_slam estimator;
    tf::TransformListener listener;
    int currentIndex;
    boost::unordered_map<std::string,int> map;
    tf::StampedTransform tf_sensor_transform;
    Affine3d sensor_transform;
    Vector3d measurement; //Holds the range bearing and elevation
    Vector3d covariance; // Holds the covariance of the above
    VectorXd position;

    
};

#endif 
