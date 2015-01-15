//#include "../include/robot.h"
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

namespace gazebo
{

/**
 * @brief class to manipulate robot behaviour
 * @author Dwijesh Bhageerutty
 */
class Robot : public ModelPlugin
{
public:

	/**
	 * Constructor
	 */
	Robot() {
		int argc = 0;
		ros::init(argc, NULL, "Robot Plugin");
		std::cout << "Robot plugin node created." << std::endl;
		noOfIterations = 0;
	};

	/**
	 * Destructor
	 */
	~Robot() {
		delete this->node;
	};

	/**
	 * Overidden.
	 */
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
		// Store the pointer to the model
		if (_parent != NULL) this->model = _parent;

		// private ROS NodeHandle
		this->node = new ros::NodeHandle("~");

		// ROS Subscriber
		this->twistSub = this->node->subscribe("simulator/robot_twist", 1000, &Robot::moveCallback, this);

		// /controls/wrench/
		this->controlsWrenchSub = this->node->subscribe("/controls/wrench", 1000, &Robot::controlsWrenchCallBack, this);

		// Marker Drop Sub topic subscriber 
		this->markerDropSub = this->node->subscribe("simulator/marker", 1000, &Robot::simulatorMarkerCallBack, this);

		// Listen to the update event. This event is broadcast every simulation iteration.
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Robot::OnUpdate, this, _1));
	};

	/**
	 * Called on every simulation iteration
	 */
	int noOfIterations;
	void OnUpdate(const common::UpdateInfo & /*_info*/) {
		++noOfIterations;

		if (noOfIterations % 500 == 0) {
			if (!applyDrag()) {
				ROS_ERROR("Drag application failed.");
			}
		}

		ros::spinOnce();
	};

	/**
	 * When a Twist message is passed to the gazebo/twist topic,
	 * the linear and angular velocities for the robot are changed.
	 * @param msg Twist message
	 */
	void moveCallback(const geometry_msgs::Twist::ConstPtr& msg) {
		this->model->SetLinearVel(math::Vector3(msg->linear.x, msg->linear.y, -msg->linear.z));
		this->model->SetAngularVel(math::Vector3(msg->angular.x, msg->angular.y, msg->angular.z));
	};

	/**
	 * Calculates the drag vector based on the robot's velocity and
	 * applies the corresponding wrench.
	 */
	bool applyDrag() {
		geometry_msgs::Wrench wrench;

		wrench.force = calculateDragForce(model->GetRelativeLinearVel());
		wrench.torque = calculateDragTorque(model->GetRelativeAngularVel());;

		if (!shouldApplyForce(wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z)) 
			return true;

		// ApplyBodyWrench message
		gazebo_msgs::ApplyBodyWrench applyBodyWrench;
		applyBodyWrench.request.body_name = (std::string) "robot::body";
		applyBodyWrench.request.wrench = wrench;
	//	applyBodyWrench.request.reference_frame = "robot::robot_reference_frame";

		//applyBodyWrench.request.start_time not specified -> it will start ASAP.
		applyBodyWrench.request.duration = ros::Duration(1);

		ros::ServiceClient client = node->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

		client.call(applyBodyWrench);

		if (!applyBodyWrench.response.success) {
			ROS_ERROR("ApplyBodyWrench call failed.");
		}
		return applyBodyWrench.response.success;
	}
	
	/**
	 * Calculate current drag force
	 * @param linearVelocity robot's current linear velocity
	 */
	geometry_msgs::Vector3 calculateDragForce(math::Vector3 linearVelocity) {
		
		float u = linearVelocity.x;
		float v = linearVelocity.y;
		float w = linearVelocity.z;
		
		float magnitude = sqrt(u*u + v*v + w*w);

		float u_unit = 0, v_unit = 0, w_unit = 0;
		if (magnitude > 1E-5) {
			u_unit = u/magnitude;
			v_unit = v/magnitude;
			w_unit = w/magnitude;
		}

		//Drag force = -0.5 * Area * density * |speed|^2 * drag coefficient
		float NEG_ONE_HALF = -0.5;
		float AREA_OF_ROBOT = .118;
		float FLUID_DENSITY = 50.0;
		float DRAG_COEFFICIENT = 0.8;
		float dragForceMagnitude = NEG_ONE_HALF * AREA_OF_ROBOT * FLUID_DENSITY
									* magnitude * magnitude * DRAG_COEFFICIENT;
		
		geometry_msgs::Vector3 dragForceVector;
		dragForceVector.x = (u_unit * dragForceMagnitude);
		dragForceVector.y = (v_unit * dragForceMagnitude);
		dragForceVector.z = (w_unit * dragForceMagnitude);
		ROS_INFO("input velocity in x: %f", u);
		ROS_INFO("output drag in x: %f", dragForceVector.x);
		ROS_INFO("input velocity in y: %f", v);
		ROS_INFO("output drag in y: %f", dragForceVector.y);
		ROS_INFO("input velocity in z: %f", w);
		ROS_INFO("output drag in z: %f", dragForceVector.z);
		return dragForceVector;
	}

	/**
	 * Calculate current drag torque
	 * @param angularVelocity robot's current angular velocity
	 */
	geometry_msgs::Vector3 calculateDragTorque(math::Vector3 angularVelocity) {
		return calculateDragForce(angularVelocity);	
/*		geometry_msgs::Vector3 dragForceVector;
		dragForceVector.x = 0;
		dragForceVector.y = 0;
		dragForceVector.z = 0;

		return dragForceVector;	
*/	}	
	
	/**
	 * Function to handle Wrench messages passed to topic 'controls/wrench/'
	 * @param msg Wrench to be applied to robot
	 */
	void controlsWrenchCallBack(const geometry_msgs::Wrench msg) {
		// check if its worth trying to apply the wrench - e.g.: if everything is 0, just return.
		if (!shouldApplyForce(msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z))
			return;

		gazebo_msgs::ApplyBodyWrench applyBodyWrench;

		applyBodyWrench.request.body_name = (std::string) "robot::body";
		applyBodyWrench.request.wrench = msg;
		applyBodyWrench.request.reference_frame = "robot::robot_reference_frame";

		//applyBodyWrench.request.start_time not specified -> it will start ASAP.
		applyBodyWrench.request.duration = ros::Duration(1);
		ros::ServiceClient client = node->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

		client.call(applyBodyWrench);
		
		if (!applyBodyWrench.response.success) {
			ROS_ERROR("ApplyBodyWrench call failed.");
		}
	}

	void simulatorMarkerCallBack(const std_msgs::Bool::ConstPtr& msg) {
		if (msg->data) {
			ROS_INFO("Dropped Marker");
		}
	}

	bool shouldApplyForce(float u, float v, float w, float p, float q, float r) {
		return (isOutOfRange(u) || isOutOfRange(v) || isOutOfRange(w) || isOutOfRange(p) || isOutOfRange(q) || isOutOfRange(r));
	}	
	
	bool isOutOfRange(float x) {
		const float RANGE_BOUND = .0001;
		return (ABS_FLOAT(x) > RANGE_BOUND);
	}

private:
	void printWrenchMsg(geometry_msgs::Wrench msg) {
		std::cout << "Applying wrench obtained from controls/wrench topic:" << std::endl;
		std::cout << "fx:" << msg.force.x << ", fy:" << msg.force.y << " fz:" << msg.force.z;
		std::cout << " taoX:" << msg.torque.x << ", taoY:" << msg.torque.y << " taoZ:" << msg.torque.z << std::endl;
	}

	// CONSTANTS

	// for angular torque (currently off, were all 1)
	const static float KP = 0;
	const static float KQ = 0;
	const static float KR = 0;

	/** Pointer to the model */
	physics::ModelPtr model;

	/** Pointer to the update event connection */
	event::ConnectionPtr updateConnection;

	/** ROS NodeHandle */
	ros::NodeHandle* node;

	/** robot_twist Subscriber */
	ros::Subscriber twistSub;
	
	/** Controls wrench topic subscriber */
	ros::Subscriber controlsWrenchSub;

	/** Marker Drop Sub topic subscriber */
	ros::Subscriber markerDropSub;

	/** Torpedo Launch Sub topic subscriber */
// suspiciously unused	
//	ros::Subscriber	torpedoLaunchSub;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Robot)
}
