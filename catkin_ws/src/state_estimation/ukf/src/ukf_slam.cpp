#include "ukf.h"
#include "ukf_slam.h"
#include <math.h>
#include <stdio.h>
#include <boost/bind.hpp>

ukf_slam::ukf_slam(int n):
  estimator(VectorXd::Zero(3*n), 100 * MatrixXd::Identity(3*n,3*n)),
  N(n)
{}

double lnRange(Vector3d v) {
  return ln(v(0)*v(0) + v(1)*v(1) + v(2)*v(2))/2.;
}

double bearing(Vector3d v) {
  return atan2(v(1), v(0));
}

double elevation(Vector3d v) {
  return atan2(v(2), sqrt(v(0)*v(0) + v(1)*v(1)));
}

MatrixXd ukf_slam::observe(int objectIndex, double base_yaw, Transformation transform, const MatrixXd sigmas) {
  //Hide all the bits we don't care about and convert to the sensor frame
  MatrixXd conv_sigmas = transform * sigmas.block(objectIndex, 0, 3, sigmas.cols());	//Removed *3 for index
  
  // Three rows because ln_range, bearing, elevation
  MatrixXd msmts = MatrixXd(3, conv_sigmas.cols());
  
  for (int i = 0; i < conv_sigmas.cols(); i++) {
    const Vector3d sigma = cov_sigmas.col(i);
    msmts(i, 0) = lnRange(sigma);
    msmts(i, 1) = (bearing(sigma) - base_yaw + M_PI) % (2*M_PI);
    msmts(i, 2) = elevation(sigma)
  }
  
  return msmts;
}	

void ukf_slam::propogate(Ref<Eigen::VectorXd> state) {
  // Does nothing because we don't have odometry implemented yet
}

MatrixXd ukf_slam::getCovariance(int objectId) {
  return estimator.covariance.block(2*objectId, 2*objectId, 2, 2);
}

void ukf_slam::update(int objectIndex, Transformation transform, const Vector3d msmt, constVector3d covar)
{
	estimator.predict(&propogate, 0.01 * MatrixXd::Identity(3*N,3*N));
	//Note that we pass in PI for the measured bearing. This is because observe take in the measured bearing and centers
	// all the sigma point's bearings around the measured bearing so we don't run into issues with the jump from 2*pi to zero
	estimator.correct(Vector3d(msmt(0),M_PI,msmt(2)), boost::bind(&ukf_slam::observe, objectIndex, msmt(1), _1),
	    covar.asDiagonal());
	//TODO: make sure this is a copy. Pretty sure it is
	return estimator.state;
}
