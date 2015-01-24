#include "ukf.h"
//#include "matrix_utils.h"
//#include "rotation_vector_utils.h"
#include <math.h>
#include <stdio.h> //TODO(max) do we need this?
//#include "ukf_pose.h"
#include <iostream> //remove before flight



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



ukf::ukf(int dim) : 
	DIM(dim), 
	state(Vector3d::Zero()),
	covariance(INITIAL_COVARIANCE * Matrix3d::Identity()),
	processCovariance(PROCESS_VARIANCE * Matrix3d::Identity()),
	measurementCovariance(MEASUREMENT_VARIANCE * Matrix3d::Identity()),
	sigmas(Matrix3X6d::Zero()),
	gammas(Matrix3X6d::Zero())

{}

void ukf::generateSigmas()
{

//This method generates 2*DIM states distributed on a hypersphere around augPose

	//Cholesky Decomposition
	Matrix3d T = covariance.llt().matrixL();

	//Concatenate both scaled Ts and add state
	sigmas << -sqrt(3.)*T , sqrt(3.)*T;
	sigmas.colwise() += state;
	if (sigmas.hasNaN()) printf("Sigmas has NaN");
}



void ukf::recoverPrediction()
{
	//Average of sigmas (3x6) stored in state (3x1)
	state = sigmas.rowwise().mean();
	//printf("\nstate\n");
	//std::cout << state;
	if (state.hasNaN()) printf("State has NaN");//TODO(max) remove all these hasNaN tests

	//TODO(max) Clean this up a bit. maybe
	Matrix3X6d Temp;
	Temp << state, state, state, state, state, state;
	//printf("\nTemp\n");
	//std::cout << Temp;
	Matrix3X6d diff(sigmas-Temp);
	//printf("\ndiff\n");
	//std::cout<<diff;
	covariance = diff*diff.transpose()/6.;
	//covariance = (1.0/6.0)*((sigmas - Temp) * (sigmas - Temp).transpose() ) + processCovariance; remove before flight
	//printf("\ncovariance\n");
	//std::cout << covariance;
	//printf("\ndone\n");
	if (covariance.hasNaN()) printf("Covariance has NaN");

}

void ukf::predict(constVector rotation, void (*propogate)(Eigen::VectorXd,Ref<Eigen::VectorXd>))
{
	generateSigmas();
	//printf("\nsigmas\n");
	//std::cout << sigmas;//remove before flight
	for (int i = 0; i < 2*DIM; i++)
	{
		propogate(rotation, sigmas.col(i));	//Call to ukf_pose
		
	}
	//printf("\npropogated sigmas\n");
	//std::cout << sigmas;//remove before flight
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
	Vector3d predMsmt = gammas.rowwise().mean();

	Matrix3X6d Temp1;
	Temp1 << state, state, state, state, state, state;
	Matrix3X6d Temp2;
	Temp2 << predMsmt, predMsmt, predMsmt, predMsmt, predMsmt, predMsmt;

	Matrix3d measCovar = ( (gammas - Temp2) * (gammas - Temp2).transpose() )/6. + measurementCovariance;
	Matrix3d crossCovar = ( (sigmas - Temp1) * (gammas - Temp2).transpose() )/6.;

	// gain = croscovar*meascovar^-1
	Matrix3d gain = measCovar.transpose().ldlt().solve(crossCovar.transpose()).transpose();


	state += gain * (acc - predMsmt);


	covariance -= crossCovar * gain.transpose();
}
