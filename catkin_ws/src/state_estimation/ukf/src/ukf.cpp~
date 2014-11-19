#include "ukf.h"
#include "matrix_utils.h"
#include "rotation_vector_utils.h"
#include <math.h>
#include <stdio.h>
#include <Eigen/Dense>

//Length of the state vector
const double INITIAL_COVARIANCE = 0.0000001;
const double PROCESS_VARIANCE = 0.00000004;
const double MEASUREMENT_VARIANCE = 0.025;

const double pi = std::acos(-1.0);

//Method to print matrix or vector
void Print(m) 
{
cout << "Printing: \n" << m << endl;
}


ukf::ukf(int dim)
{
	Vector3d state;
	Matrix3d covarianceMatrix;
	Matrix3X6d sigmas;
	Matrix3X6d gammas;
	Vector3d predMsmt;
	Matrix3d measCovar;
	Matrix3d crossCovar;

	Vector3d covarianceValues(INITIAL_COVARIANCE,INITIAL_COVARIANCE,INITIAL_COVARIANCE);
	covarianceMatrix = covarianceValues.asDiagonal();
}

void ukf::generateSigmas()
{

//This method generates 2*DIM states distributed on a hypersphere around augPose

	//Replaces Cholesky method
	sigmas= augCovar.llt(); 

	//Initialize temporary matrix to have size 3 by 6
	MatrixXd T(sigmas.rows(), 2*sigmas.rows()); 

	//Concatenate both scaled sigmas to give T
	T << sigmas.scale(-sqrt(3)) , sigmas.scale(sqrt(3)) ;
}


void propogate(double *rotation, double* state)
{
	//double rotationEarth[3] = {-rotation[0], -rotation[1], -rotation[2]};
	double result[3] = {};
	double tau = 2*pi;

	composeRotations(rotation, state, result);

	double angle = norm(result);
	if (angle != 0)
	{
		scaleVector(1/angle, result, 3);
	}

	//We want to choose the rotation vector closest to sigma
	angle += tau*floor(0.5 + (dot(state, result)-angle)/tau);

	for (int j = 0; j < 3; j++)
	{
		state[j] = angle * result[j];
	}
}

void ukf::recoverPrediction()
{
	//Average of sigmas (3x6) stored in augState (3x1)
	augState = sigmas.rowwise().mean();
	augCovar = 

//SMALL PROBLEM: In order to transpose a matrix need the size to be resizeable!!! 





//Subtraction:

	Matrix3X6d tempSigmas;
	tempSigmas << augState, augState, augState, augState, augState, augState;

	subtractMultipleVectors(sigmas, augState, 2*DIM, DIM);

	averageOuterProduct(sigmas, sigmas, augCovar
			,2*DIM, DIM, DIM);

	//We add process_variance to each diagonal of the augCovar matrix
	Vector3d processVarianceAsVector (PROCESS_VARIANCE, PROCESS_VARIANCE, PROCESS_VARIANCE);
	processVarianceAsMatrix = processVarianceAsVector.asDiagonal();

	augCovar = augCovar + processVarianceAsMatrix;;
}

void ukf::predict(double rotation[3])
{
	generateSigmas();
	//prettyPrint(sigma(0), 2*DIM, DIM);
	for (int i = 0; i < 2*DIM; i++)
	{
		propogate(rotation, sigma(i));
	}
	//prettyPrint(sigma(0), 2*DIM, DIM);
	//prettyPrint(augCovar, DIM, DIM);
	recoverPrediction();
	//prettyPrint(augCovar, DIM, DIM);
}

void h(double *sigma, double *gamma)
{
	double gravity[] = {0, 0, 9.8};
	double inverted[3] = {};
	inverse(sigma, inverted);
	rotateThisByThat(gravity, inverted, gamma);
}

void ukf::correct(double acc[3])
{
	generateSigmas();

	//prettyPrint(sigma(0), 2*DIM, DIM);

	//Here we predict the outcome of the acceleration measurement
	//for every sigma and store in the gammas
	for (int i = 0; i < 2* DIM; i++)
	{
		h(sigma(i), gamma(i));
	}

	//prettyPrint(gamma(0), 2*DIM, DIM);
	recoverCorrection(acc);
}

void ukf::recoverCorrection(double *acc)
{

	averageVectors(gammas, predMsmt, 2*DIM, DIM);

	subtractMultipleVectors(sigmas, augState, 2*DIM, DIM);
	subtractMultipleVectors(gammas, predMsmt, 2*DIM, DIM);

	averageOuterProduct(gammas, gammas, measCovar, 2*DIM, DIM, DIM);
	averageOuterProduct(sigmas, gammas, crossCovar, 2*DIM, DIM, DIM);


	//Include measurement variance
	addDiagonal(measCovar, MEASUREMENT_VARIANCE, DIM);


	//prettyPrint(measCovar, DIM, DIM);
	//prettyPrint(crossCovar, DIM, DIM);


	double *gain = new double[DIM*DIM]();
    solve(crossCovar, measCovar, gain, DIM, DIM);


    subtractVectors(acc, predMsmt, DIM);
    leftMultiplyAdd(gain, acc, augState, DIM, DIM, 1);

    //prettyPrint(gain, DIM, DIM);

    scaleVector(-1.0, crossCovar, DIM*DIM);
    transposedMultiplyAdd(crossCovar, gain, augCovar, DIM, DIM, DIM);
    //prettyPrint(augCovar, DIM, DIM);

    delete gain;
}

void fixState(double* state)
{
	double angle = norm(state);
	if (angle > pi)
	{
		//This represents the same rotation, but with norm < pi
		scaleVector(-(2*pi-angle)/angle, state, 3);
	}
}


void ukf::update(double* acc, double* rotation, double *quaternion)
{
	//printf("%3f %3f\n", rotation[1], augState[1]);
	fixState(augState);

    predict(rotation);
    correct(acc);

    quaternionFromRotationVector(quaternion, augState);
}

double *ukf::sigma(int index)
{//Returns a pointer to the desired sigma array
	return vectorIndex(sigmas, index, DIM);
}

double *ukf::gamma(int index)
{//Returns a pointer to the desired sigma array
	return vectorIndex(gammas, index, DIM);
}

/*int main()
{
	ukf filter(3);
	double gyro [3] = {};
	filter.update(gyro, gyro);
	return 0;
}*/

