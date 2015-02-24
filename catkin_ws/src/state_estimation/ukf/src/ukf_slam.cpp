#include "ukf.h"
#include "ukf_slam.h"
#include <math.h>
#include <stdio.h>
#include <boost/bind.hpp>

ukf_slam::ukf_slam():
  estimator(VectorXd::Zero(2), 100 * MatrixXd::Identity(2,2))
{}


MatrixXd ukf_slam::observe(MatrixXd sigmas, int objectID) {
	return sigmas.block(2*objectID,0, 2, 2*sigmas.rows());	//Return 2 by 4n block from sigmas
}	

void ukf_slam::propogate(Ref<Eigen::VectorXd> state) {
  // Does nothing because we don't have odometry implemented yet
}


void ukf_slam::update(Vector2d msmt, Ref<Vector2d> outPosition, int objectID)
{
	estimator.predict(&propogate, 0.1 * MatrixXd::Identity(2,2));
	estimator.correct(msmt, boost::bind(&ukf_slam::observe, _1, objectID), MatrixXd::Identity(2,2));
	outPosition = estimator.state;
}
