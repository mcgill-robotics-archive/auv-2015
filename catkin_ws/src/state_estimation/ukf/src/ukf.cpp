#include "ukf.h"
//#include "matrix_utils.h"
//#include "rotation_vector_utils.h"
#include <math.h>
#include <stdio.h>
//#include "ukf_pose.h"



//Length of the state vector
const double INITIAL_COVARIANCE = 0.0000001;
const double PROCESS_VARIANCE = 0.00000004;
const double MEASUREMENT_VARIANCE = 0.025;

const double pi = std::acos(-1.0);

//ukf_pose ukfpose;

//Method to print matrix or vector
/*void Print(MatrixXd m) 
{
cout << "Printing: \n" << m << endl;
}*/



ukf::ukf(int dim)
{
	DIM = dim;
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



void ukf::recoverPrediction()
{
	//Average of sigmas (3x6) stored in state (3x1)
	state = sigmas.rowwise().mean();
	Matrix3X6d Temp;
	Temp << state, state, state, state, state, state;

	covariance = ((sigmas - Temp) * (sigmas - Temp).transpose() ) - processCovariance;

}

void ukf::predict(constVector rotation, void (*propogate)(Eigen::VectorXd,Ref<Eigen::VectorXd>))
{
	generateSigmas();
	Vector3d gravity(0,0,0);
	for (int i = 0; i < 2*DIM; i++)
	{
		propogate(rotation, sigmas.col(i));	//Call to ukf_pose
		
	}
	recoverPrediction();
	
}


void ukf::correct(constVector acc, void (*observe)(Eigen::VectorXd,Ref<Eigen::VectorXd>))
{
	generateSigmas();
	//Here we predict the outcome of the acceleration measurement
	//for every sigma and store in the gammas
	for (int i = 0; i < 2* DIM; i++)
	{	
		observe(sigmas.col(i), gammas.col(i));	//Call to ukf_pose
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
