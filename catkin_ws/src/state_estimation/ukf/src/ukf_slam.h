#ifndef UKF_SLAM_
#define UKF_SLAM_

#include <eigen3/Eigen/Dense>
#include "ukf.h"

using namespace Eigen;

class ukf_slam
{
	public:
    ukf_slam(int); 
    VectorXd update(int objectID, const Affine3d transform, const Vector3d msmt,
        const Vector3d covar);
		void static propogate(Ref<Eigen::VectorXd>);
    MatrixXd static observe(int objectId, double base_yaw, const Affine3d transform,
        const MatrixXd sigmas);
		MatrixXd getCovariance(int objectId);
		
	private:
		ukf estimator;
		const int N;
};

#endif
