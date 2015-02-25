#include "ukf.h"
#include "ukf_slam.h"
#include <math.h>
#include <stdio.h>
#include <boost/bind.hpp>

ukf_slam::ukf_slam(int n):
  estimator(VectorXd::Zero(2*n), 100 * MatrixXd::Identity(2*n,2*n))
{}


MatrixXd ukf_slam::observe(MatrixXd sigmas, int objectID) {
  //Return 2 by 4n block from sigmas
	return sigmas.block(2*objectID,0, 2, sigmas.cols());	
}	

void ukf_slam::propogate(Ref<Eigen::VectorXd> state) {
  // Does nothing because we don't have odometry implemented yet
}

MatrixXd ukf_slam::getCovariance(int objectId) {
  return estimator.covariance.block(2*objectId, 2*objectId, 2, 2);
}

void ukf_slam::update(Vector2d msmt, Ref<VectorXd> outPosition, int objectID)
{
	estimator.predict(&propogate, 0.0 * MatrixXd::Identity(2*4,2*4));
	estimator.correct(msmt, boost::bind(&ukf_slam::observe, _1, objectID),
	    MatrixXd::Identity(2,2));
	outPosition = estimator.state;
}
