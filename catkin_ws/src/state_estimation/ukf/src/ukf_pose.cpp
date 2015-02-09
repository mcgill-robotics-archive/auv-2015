#include "ukf_pose.h"
#include "ukf.h"
#include <math.h>
#include <stdio.h>
#include <iostream>



//Length of the state vector
//const double INITIAL_COVARIANCE = 0.0000001;
//const double PROCESS_VARIANCE = 0.00000004;
//const double MEASUREMENT_VARIANCE = 0.025;

const double pi = std::acos(-1.0);
const int DIM = 3;

//Method to print matrix or vector
/*void Print(MatrixXd m) 
{
cout << "Printing: \n" << m << endl;
}*/

ukf_pose::ukf_pose():estimator(3) {}

AngleAxisd ukf_pose::angleAxis(Vector3d v) {
	if (v.norm() > 0) {
		return AngleAxisd(v.norm(), v.normalized());
	} else {
		return AngleAxisd(0., Vector3d::UnitZ());
	}
}

void ukf_pose::propogate(Eigen::Vector3d rotation, Ref<Eigen::Vector3d> state)	//Use boost bind
{
	
	//double rotationEarth[3] = {-rotation[0], -rotation[1], -rotation[2]};
	
	//double result[3] = {};
	double tau = 2*pi;

	AngleAxisd rotationTransform(angleAxis(rotation));
	AngleAxisd stateTransform(angleAxis(state));	
	AngleAxisd resultTransform(stateTransform * rotationTransform);	

	//We want to choose the rotation vector closest to sigma
	double angle = resultTransform.angle() + tau * floor(0.5 + (state.dot(resultTransform.axis())-resultTransform.angle())/tau);

	state = angle * resultTransform.axis();
	if (state.hasNaN()) printf("Sigma has nan");
}

//Return gammas, take in full sigmas, cycle through cols in here
Matrix3X6d ukf_pose::observe(Matrix3X6d sigmas)	//Returns gammas
{
	Vector3d gravity(0,0,9.8);
	Matrix3X6d gammas;
	for(int i = 0; i < 2*DIM; i++)
	{
	//double inverted[3] = {};
	//inverse(sigma, inverted);
	//rotateThisByThat(gravity, inverted, gamma);
		AngleAxisd invertedAngleAxis(angleAxis(-sigmas.col(i)));

	//Transform invertedTransform(invertedAngleAxis);
		gammas.col(i) = invertedAngleAxis.toRotationMatrix() * gravity; //Apply the transform to gravity
	}
	return gammas;
}

void ukf_pose::update(constVector3 acc, constVector3 rotation, Ref<Vector3d> pose) //acc as constVector ?
{
    fixState(estimator.state);

    //estimator.predict(rotation, &propogate);	//Todo : rotation should be accessed from estimator, not passed
    estimator.predict(&propogate);
    estimator.correct(acc, &observe);

    pose = estimator.state;
}

void ukf_pose::fixState(Ref<Vector3d> state)
{
	double angle = state.norm();
	if (angle > pi)
	{
        state = (angle - 2*pi)*state.normalized(); //TODO(max) better reference for pi
	}
}
	


