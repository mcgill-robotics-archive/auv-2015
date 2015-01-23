#include "ukf_pose.h"
#include "ukf.h"
#include <math.h>
#include <stdio.h>



//Length of the state vector
//const double INITIAL_COVARIANCE = 0.0000001;
//const double PROCESS_VARIANCE = 0.00000004;
//const double MEASUREMENT_VARIANCE = 0.025;

const double pi = std::acos(-1.0);


//Method to print matrix or vector
/*void Print(MatrixXd m) 
{
cout << "Printing: \n" << m << endl;
}*/

ukf_pose::ukf_pose():estimator(3) {}


void ukf_pose::propogate(Eigen::VectorXd rotation, Ref<Eigen::VectorXd> state)
{
	
	//double rotationEarth[3] = {-rotation[0], -rotation[1], -rotation[2]};
	
	//double result[3] = {};
	Vector3d result(0,0,0);
	double tau = 2*pi;

	
	
	//Construct transforms as AngleAxis' and compose the transforms
	AngleAxisd rotationTransform(rotation.norm(), rotation.normalized()); //Rotation as AngleAxis
	AngleAxisd stateTransform(state.norm(), state.normalized());
	AngleAxisd resultTransform(rotationTransform * stateTransform);
	
	
	result = resultTransform.angle()*resultTransform.axis(); //Get a vector back	

	double angle = result.norm();
	result = result.normalized();

	//We want to choose the rotation vector closest to sigma
	angle += tau *floor(0.5 + (state.dot(result)-angle)/tau);

	state = angle * result;	
}

void ukf_pose::observe(VectorXd sigma, Ref<VectorXd> gamma)
{
	Vector3d gravity(0,0,9.8);

	Vector3d inverted = -sigma;
	//double inverted[3] = {};
	//inverse(sigma, inverted);
	//rotateThisByThat(gravity, inverted, gamma);
	AngleAxisd invertedAngleAxis(inverted.norm(), inverted.normalized());
	//Transform invertedTransform(invertedAngleAxis);
	gamma = invertedAngleAxis.toRotationMatrix() * gravity; //Apply the transform to gravity

}

void ukf_pose::update(constVector acc, constVector rotation, Ref<Vector3d> pose) //acc as constVector ?
{
    fixState(estimator.state);

    estimator.predict(rotation, &propogate);
    //estimator.correct(acc, &observe);

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
	


