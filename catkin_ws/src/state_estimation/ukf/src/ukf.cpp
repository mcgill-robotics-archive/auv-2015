#include "ukf.h"
#include <math.h>


//Length of the state vector
const double INITIAL_COVARIANCE = 0.0000001;
const double PROCESS_VARIANCE = 0.00000004;
const double MEASUREMENT_VARIANCE = 0.025;

const double pi = std::acos(-1.0);


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
	Matrix3d T = covariance.llt().matrixL();//TODO(max) do we need this temp matrix?

	//Concatenate both scaled Ts and add state
	sigmas << -sqrt(3.)*T , sqrt(3.)*T;
	sigmas.colwise() += state;
}



void ukf::recoverPrediction()
{
	//Average of sigmas (3x6) stored in state (3x1)
	state = sigmas.rowwise().mean();

	//TODO(max) Clean this up a bit. maybe
	Matrix3X6d Temp;
	Temp << state, state, state, state, state, state;
	Matrix3X6d diff(sigmas-Temp);
	covariance = diff*diff.transpose()/6.;

}

void ukf::predict(void (*propogate)(Eigen::VectorXd,Ref<Eigen::VectorXd>))
{
	generateSigmas();
	for (int i = 0; i < 2*DIM; i++)
	{
		propogate(sigmas.col(i));//TODO(max) Better way to do this with colwise? rotation was removed as an arg
		
	}
	recoverPrediction();
}


void ukf::correct(constVector measurement, void (*observe)(Eigen::VectorXd,Ref<Eigen::VectorXd>))
{
	generateSigmas();
	for (int i = 0; i < 2* DIM; i++)
	{	
		observe(sigmas.col(i), gammas.col(i));
	}
	recoverCorrection(measurement);
}

void ukf::recoverCorrection(constVector measurement)
{
	Vector3d predMsmt = gammas.rowwise().mean();

	//TODO(max) we can replace these with colwise
	Matrix3X6d Temp1;
	Temp1 << state, state, state, state, state, state;
	Matrix3X6d Temp2;
	Temp2 << predMsmt, predMsmt, predMsmt, predMsmt, predMsmt, predMsmt;

	Matrix3d measCovar = ( (gammas - Temp2) * (gammas - Temp2).transpose() )/6. + measurementCovariance;
	Matrix3d crossCovar = ( (sigmas - Temp1) * (gammas - Temp2).transpose() )/6.;

	// gain = croscovar*meascovar^-1
	Matrix3d gain = measCovar.transpose().ldlt().solve(crossCovar.transpose()).transpose();

	state += gain * (measurement - predMsmt);
	covariance -= crossCovar * gain.transpose();
}
