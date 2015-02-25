#ifndef UKF_SLAM_
#define UKF_SLAM_

#include <eigen3/Eigen/Dense>
#include "ukf.h"

using namespace Eigen;

class ukf_slam
{
	public:
    ukf_slam(int); 
    void update(Vector2d, Ref<VectorXd>, int);
		//void propogate(Vector3d, Ref<Vector3d>);
		void static propogate(Ref<Eigen::VectorXd>);
		//void h(Vector3d, Ref<Vector3d>);
		MatrixXd static observe(MatrixXd, int);
		MatrixXd getCovariance(int objectId);
		
	private:
		ukf estimator;
};

#endif
