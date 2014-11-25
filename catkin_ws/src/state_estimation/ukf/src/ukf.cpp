#include "ukf.h"
#include "matrix_utils.h"
#include "rotation_vector_utils.h"
#include <math.h>
#include <stdio.h>


//Length of the state vector
const double INITIAL_COVARIANCE = 0.0000001;
const double PROCESS_VARIANCE = 0.00000004;
const double MEASUREMENT_VARIANCE = 0.025;

const double pi = std::acos(-1.0);

//Method to print matrix or vector
/*void Print(MatrixXd m) 
{
cout << "Printing: \n" << m << endl;
}*/



ukf::ukf(int dim)
{

	Vector3d covarianceValues(INITIAL_COVARIANCE,INITIAL_COVARIANCE,INITIAL_COVARIANCE);

	covariance = covarianceValues.asDiagonal();

	Matrix2d processVarianceMatrix;

	processVarianceMatrix << PROCESS_VARIANCE, PROCESS_VARIANCE,
	PROCESS_VARIANCE, PROCESS_VARIANCE;


}

void ukf::generateSigmas()
{

//This method generates 2*DIM states distributed on a hypersphere around augPose

	//Cholesky Decomposition
	Matrix3d T = covariance.llt().matrixL();

	//Concatenate both scaled sigmas to give T
	sigmas << (-sqrt(3))*T , sqrt(3)*T;
}


void propogate(constVector rotation, constVector state)
{
	//double rotationEarth[3] = {-rotation[0], -rotation[1], -rotation[2]};
	
	//double result[3] = {};
	Vector3d result(0,0,0);
	double tau = 2*pi;

	//Get rotation and state as AngleAxis'
	AngleAxisf rotationAngleAxis = AngleAxisf(rotation.norm(), rotation.normalize());
	AngleAxisf stateAngleAxis = AngleAxisf(state.norm(), state.normalize());
	
	//Construct transforms from AngleAxis' and compose the transforms
	Transform rotationTransform(rotationAngleAxis);
	Transform stateTransform(stateAngleAxis);
	Transform resultTransform = rotationTransform * stateTransform;
	
	Matrix3d resultMatrix = resultTransform.rotation(); //Convert to rotation matrix
	AngleAxisf resultAngleAxis = AngleAxisf(resultMatrix); //Convert to AngleAxis representation
	result = resultAngleAxis.angle()*resultAngleAxis.axis(); //Get a vector back
	
	//composeRotations(rotation, state, result);
	

	double angle = result.norm();
	if (angle != 0)
	{
		result = (1/angle)*result
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

void ukf::recoverPrediction()
{
	//Average of sigmas (3x6) stored in state (3x1)
	state = sigmas.rowwise().mean();
	Matrix3X6d Temp;
	Temp << state, state, state, state, state, state;

	covariance = ((sigmas - Temp) * (sigmas - Temp).transpose() ) - processCovariance;

}

void ukf::predict(constVector rotation)
{
	generateSigmas();

	for (int i = 0; i < 2*DIM; i++)
	{
		propogate(rotation, sigmas.col(i));
	}
	recoverPrediction();
}




void h(double *sigma, double *gamma)
{/*
	double gravity[] = {0, 0, 9.8};
	double inverted[3] = {};
	inverse(sigma, inverted);
	rotateThisByThat(gravity, inverted, gamma);
*/
}

void ukf::correct(constVector acc)
{
	generateSigmas();
	//Here we predict the outcome of the acceleration measurement
	//for every sigma and store in the gammas
	for (int i = 0; i < 2* DIM; i++)
	{
	//	h(sigma(i), gamma(i));
	}

	//prettyPrint(gamma(0), 2*DIM, DIM);
	recoverCorrection(acc);
}

void ukf::recoverCorrection(constVector acc)
{
	predMsmt = gammas.rowwise().mean();

	Matrix3X6d Temp1;
	Temp1 << state, state, state, state, state, state;
	Matrix3X6d Temp2;
	Temp2 << predMsmt, predMsmt, predMsmt, predMsmt, predMsmt, predMsmt;

	measCovar = ( (sigmas - Temp1) * (sigmas - Temp1).transpose() ) + measurementCovariance;
	crossCovar = ( (sigmas - Temp2) * (gammas - Temp2).transpose() );

	// double *gain = new double[DIM*DIM]();
	Matrix3d gain = measCovar.ldlt().solve(crossCovar);


	state = gain * (acc - predMsmt);


	covariance -= crossCovar * gain.transpose();
}

void fixState(double* state)
{
	/*double angle = norm(state);
	if (angle > pi)
	{

	state.scale(-(2*pi-angle)/angle);
		//This represents the same rotation, but with norm < pi
		//scaleVector(-(2*pi-angle)/angle, state, 3);
	}*/
}


void ukf::update(constVector acc, constVector rotation, double *quaternion)
{
	//fixState(state);

    predict(rotation);
    correct(acc);

    //quaternionFromRotationVector(quaternion, state);
}


