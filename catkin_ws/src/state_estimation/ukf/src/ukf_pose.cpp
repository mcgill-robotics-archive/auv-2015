#include "ukf_pose.h"
#include "ukf.h"
//#include "matrix_utils.h"
//#include "rotation_vector_utils.h"
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

ukf_pose::ukf_pose():estimator(3)
{
	
}



//void ukf_pose::propogate(Vector3d rotation, Ref<Vector3d> state)
void ukf_pose::propogate(Eigen::VectorXd rotation, Ref<Eigen::VectorXd> state)
{
	
	//double rotationEarth[3] = {-rotation[0], -rotation[1], -rotation[2]};
	
	//double result[3] = {};
	Vector3d result(0,0,0);
	double tau = 2*pi;

	
	
	//Construct transforms as AngleAxis' and compose the transforms
	Vector3d normalizedRotation = rotation.normalized();	//Normalize vector to construct AngleAxis
	AngleAxisd rotationTransform(rotation.norm(), normalizedRotation); //Rotation as AngleAxis
	Vector3d normalizedState = state.normalized();	//Normalize vector to construct AngleAxis
	AngleAxisd stateTransform(state.norm(), normalizedState);
	AngleAxisd resultTransform(rotationTransform * stateTransform);
	
	
	result = resultTransform.angle()*resultTransform.axis(); //Get a vector back
	
	//composeRotations(rotation, state, result);
	

	double angle = result.norm();
	if (angle != 0)
	{
		result = (1/angle)*result;
		//scaleVector(1/angle, result, 3);
	}

	//We want to choose the rotation vector closest to sigma
	angle += tau*floor(0.5 + (state.dot(result)-angle)/tau);

	state = angle * result;
	
	/*for (int j = 0; j < 3; j++)
	{
		state[j] = angle * result[j];
	}*/
	
}

//void ukf_pose::h(Vector3d sigma, Ref<Vector3d> gamma)
void ukf_pose::observe(VectorXd sigma, Ref<VectorXd> gamma)
{
	Vector3d gravity(0,0,9.8);

	Vector3d inverted = -sigma;
	//double inverted[3] = {};
	//inverse(sigma, inverted);
	//rotateThisByThat(gravity, inverted, gamma);
	Vector3d normalizedInverted = inverted.normalized();
	AngleAxisd invertedAngleAxis(inverted.norm(), normalizedInverted);
	//Transform invertedTransform(invertedAngleAxis);
	gamma = invertedAngleAxis * gravity; //Apply the transform to gravity

}

void ukf_pose::update(constVector acc, constVector rotation, double *quaternion) //acc as constVector ?
{
	fixState(estimator.state);

    estimator.predict(rotation, &propogate);
    estimator.correct(acc, &observe);

    //quaternionFromRotationVector(quaternion, state);
}

void ukf_pose::fixState(Ref<Vector3d> state)
{
	double angle = state.norm();
	if (angle > pi)
	{

	
	state = state * (-(2*pi-angle)/angle);	//Scales it
		//This represents the same rotation, but with norm < pi
		//scaleVector(-(2*pi-angle)/angle, state, 3);
	}
}
	


