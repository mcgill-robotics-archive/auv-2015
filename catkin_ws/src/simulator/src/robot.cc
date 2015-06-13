#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"

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
	Robot() : controlsForce(0., 0., 0.), controlsTorque(0., 0., 0.) {
		int argc = 0; // We need to declare the int here so it can be cast to a reference in the next line.
		ros::init(argc, NULL, "Robot Plugin");
		ros::NodeHandle n;

		// Need to hold reference to subscription or else it dies
		this->controlsWrenchSub = n.subscribe("/controls/wrench", 1000, &Robot::controlsWrenchCallBack, this);
	};

	/**
	 * Overidden.
	 */
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
		// Store the pointer to the model
		this->model = _parent;


		// Listen to the update event. This event is broadcast every simulation iteration.
		// We have to store the pointer to the connection or else the connection dies.
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Robot::OnUpdate, this, _1));
	};

	/**
	 * Called on every simulation iteration
	 */
	void OnUpdate(const common::UpdateInfo &) {
		applyWrenches();
		ros::spinOnce();
	};


	/**
	 * Calculates the drag vector based on the robot's velocity and
	 * applies the corresponding wrench and the controls wrench
	 */
	bool applyWrenches() {
		model->GetLink()->AddRelativeForce(controlsForce + calculateDrag(model->GetRelativeLinearVel(), 5, 2));
		model->GetLink()->AddRelativeTorque(controlsTorque + calculateDrag(model->GetRelativeAngularVel(), 2.5, 1));
	}
	
	/**
	 * Calculate the drag using the model: f = - A*|v|v - Bv.
	 */
	math::Vector3 calculateDrag(math::Vector3 v, double A, double B) {
		return -(A * v.GetLength() + B) * v;
	}
	
	
	/**
	 * Function to handle Wrench messages passed to topic 'controls/wrench/'
	 * @param msg Wrench to be applied to robot
	 */
	void controlsWrenchCallBack(const geometry_msgs::Wrench msg) {
		controlsForce = math::Vector3(msg.force.x, msg.force.y, msg.force.z);
		controlsTorque = math::Vector3(msg.torque.x, msg.torque.y, msg.torque.z);
	}

private:	
	math::Vector3 controlsForce;
	math::Vector3 controlsTorque;

	/** Pointer to the model */
	physics::ModelPtr model;

	/** Pointer to the update event connection */
	event::ConnectionPtr updateConnection;
	
	/** Controls wrench topic subscriber */
	ros::Subscriber controlsWrenchSub;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Robot)
}
