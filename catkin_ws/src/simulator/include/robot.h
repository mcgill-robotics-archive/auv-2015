#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ApplyBodyWrench.h" 
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Wrench.h"
#include <unistd.h>
#include "std_msgs/Bool.h"

#define ABS_FLOAT(x) ((x < 0) ? -x : x)

/**
 * @brief class to manipulate robot behaviour
 * @author Dwijesh Bhageerutty
 */
namespace gazebo {

class Robot : public ModelPlugin {

public:
	
	Robot();

	~Robot();
	
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
	
	void OnUpdate(const common::UpdateInfo & /*_info*/);
	
	void moveCallback(const geometry_msgs::Twist::ConstPtr& msg);
		
	bool applyDrag();	
	
	geometry_msgs::Vector3 calculateDragForce(math::Vector3 linearVelocity);
	
	geometry_msgs::Vector3 calculateDragTorque(math::Vector3 angularVelocity);	

	void controlsWrenchCallBack(const geometry_msgs::Wrench msg);
	
	void simulatorMarkerCallBack(const std_msgs::Bool::ConstPtr& msg);

	bool shouldApplyForce(float u, float v, float w, float p, float q, float r);	

	bool isOutOfRange(float x);

private:

	void printWrenchMsg(geometry_msgs::Wrench msg);

	// CONSTANTS

	// for angular drag computation
	const static float KP;
	const static float KQ;
	const static float KR;

	/** Pointer to the model */
	physics::ModelPtr model;

	/** Pointer to the update event connection */
	event::ConnectionPtr updateConnection;

	/** ROS NodeHandle */
	ros::NodeHandle* node;

	/** robot_twist Subscriber */
	ros::Subscriber twistSub;
	
	/** ThrusterForces Subscriber */
	ros::Subscriber thrusterForcesSub;
	
	/** Controls wrench topic subscriber */
	ros::Subscriber controlsWrenchSub;

	/** Marker Drop Sub topic subscriber */
	ros::Subscriber markerDropSub;

	/** Torpedo Launch Sub topic subscriber */
	ros::Subscriber	torpedoLaunchSub;

	/** number of times the simulator has updated itself **/
	int noOfIterations;

};
//GZ_REGISTER_MODEL_PLUGIN(Robot)
}
