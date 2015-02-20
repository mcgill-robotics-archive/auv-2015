#include "ukf.h"
#include <math.h>

//Length of the state vector
const double INITIAL_COVARIANCE = 0.0000001;
const double PROCESS_VARIANCE = 0.00000004;
const double MEASUREMENT_VARIANCE = 0.025;

const double pi = std::acos(-1.0);


ukf::ukf(int dim) : 
  DIM(dim), 
  state(VectorXd::Zero(dim)),
  covariance(INITIAL_COVARIANCE * MatrixXd::Identity(dim, dim)),
  processCovariance(PROCESS_VARIANCE * MatrixXd::Identity(dim, dim)),
  measurementCovariance(MEASUREMENT_VARIANCE * MatrixXd::Identity(dim, dim)),
  sigmas(MatrixXd::Zero(dim, 2*dim))
{}

void ukf::generateSigmas()
{
  //This method generates 2*DIM states distributed on a hypersphere around augPose

  //Cholesky Decomposition
  //Matrix3d T = covariance.llt().matrixL();//TODO(max) do we need this temp matrix?
  MatrixXd T = covariance.llt().matrixL();
  //Concatenate both scaled Ts and add state
  sigmas << -sqrt((double) DIM)*T, sqrt((double) DIM)*T;  //
  sigmas.colwise() += state;
}

void ukf::recoverPrediction()
{
  state = sigmas.rowwise().mean();
  sigmas.colwise() -= state;
  covariance = sigmas*sigmas.transpose()/(2*(double) DIM);
}

void ukf::predict(boost::function<void (Ref<Eigen::VectorXd>)> propogate)
{
  generateSigmas();
  for (int i = 0; i < 2*DIM; i++)
  {
    propogate(sigmas.col(i));
  }
  recoverPrediction();
}

void ukf::correct(constVector measurement, MatrixXd(*observe)(Eigen::MatrixXd))
{
  generateSigmas();
  recoverCorrection(measurement, observe(sigmas));
}

void ukf::recoverCorrection(constVector measurement, MatrixXd gammas)
{
  VectorXd predMsmt = gammas.rowwise().mean();

  sigmas.colwise() -= state;
  gammas.colwise() -= predMsmt;
  Matrix3d measCovar = gammas * gammas.transpose()/(2. * DIM) + measurementCovariance;
  Matrix3d crossCovar = sigmas * gammas.transpose()/(2. * DIM);

  // gain = croscovar*meascovar^-1
  //Matrix3d gain = measCovar.transpose().ldlt().solve(crossCovar.transpose()).transpose();
  MatrixXd gain = measCovar.transpose().ldlt().solve(crossCovar.transpose()).transpose();

  state += gain * (measurement - predMsmt);
  covariance -= crossCovar * gain.transpose();
}
