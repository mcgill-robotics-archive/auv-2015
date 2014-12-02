#ifndef UKF_H_
#define UKF_H_

#include <eigen3/Eigen/Dense>

using namespace Eigen;

typedef Matrix<double, 3, 6> Matrix3X6d;
typedef const Ref<const Vector3d> constVector;

void propogate(Vector3d, Ref<Vector3d>);
void h(Vector3d, Ref<Vector3d>);


class ukf
{
	public:
    	ukf(int dim);
    	void update(constVector acc, constVector rotation, double *quaternion);

	private:
    	int DIM;
    	Vector3d state;
	Matrix3d covariance;
	Matrix3X6d sigmas;
	Matrix3X6d gammas;
	Vector3d predMsmt;
	Matrix3d measCovar;
	Matrix3d crossCovar;
	Matrix3d processCovariance;//TODO: Initialize this matrix
	Matrix3d measurementCovariance;//And this one too
    	void predict(constVector);
    	void correct(constVector);
    	void generateSigmas();
    	void recoverPrediction();
    	void recoverCorrection(constVector);


};


#endif /* UKF_H_ */
