#include "ukf_velocity.h"
#include "ukf.h"
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <boost/bind.hpp>

const double INITIAL_COVARIANCE = 0.0000001;
const double PROCESS_VARIANCE = 0.00000004;
const double MEASUREMENT_VARIANCE = 0.025;

ukf_velocity::ukf_velocity() :
	estimator(VectorXd::Zero(2), INITIAL_COVARIANCE * MatrixXd::Identity(2,2)), 
	processNoise(PROCESS_VARIANCE * MatrixXd::Identity(2,2)),
	measurementNoise(MEASUREMENT_VARIANCE * MatrixXd::Identity(2,2))
{}
	
void ukf_velocity::propogate(const Vector2d acc, float dt, Ref<Vector2d> state) {
	state =  state + acc * dt;
}

MatrixXd ukf_velocity::observe(MatrixXd sigmas) {
	return sigmas;
}		

void ukf_velocity::update(Vector2d acc, float dt, Ref<Vector2d> outVelocity){
	estimator.predict(boost::bind(&ukf_velocity::propogate, acc, dt, _1), processNoise);
	estimator.correct(acc, &observe, measurementNoise);
	outVelocity = estimator.state;
}	
