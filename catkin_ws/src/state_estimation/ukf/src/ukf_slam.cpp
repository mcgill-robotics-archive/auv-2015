#include "ukf.h"
#include "ukf_slam.h"
#include <math.h>
#include <stdio.h>

ukf_slam::ukf_slam(int n):
  estimator(VectorXd::Zero(2*n), 100 * MatrixXd::Identity(2*n,2*n))
{}


MatrixXd ukf_slam::observe(MatrixXd sigmas) {
	return sigmas;
}	

void ukf_slam::propogate(Ref<Eigen::VectorXd> state) {
  // Does nothing because we don't have odometry implemented yet
}


void ukf_slam::update(Vector2d msmt, Ref<Vector2d> outPosition)
{
	estimator.predict(&propogate, 0.1 * MatrixXd::Identity(2,2));
	estimator.correct(msmt, &observe, MatrixXd::Identity(2,2));
	outPosition = estimator.state;
}