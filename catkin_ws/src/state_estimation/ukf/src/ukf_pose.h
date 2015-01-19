#ifndef UKF_POSE_
#define UKF_POSE_

#include <eigen3/Eigen/Dense>
#include "ukf.h"

using namespace Eigen;

typedef Matrix<double, 3, 6> Matrix3X6d;
typedef const Ref<const Vector3d> constVector;



class ukf_pose
{
	public:
    	ukf_pose(); 
    	void update(constVector acc, constVector rotation, double *quaternion);
		//void propogate(Vector3d, Ref<Vector3d>);
		void static propogate(Eigen::VectorXd, Ref<Eigen::VectorXd>);
		//void h(Vector3d, Ref<Vector3d>);
		void static observe(Eigen::VectorXd, Ref<Eigen::VectorXd>);
		void fixState(Ref<Vector3d>);

	private:
		ukf estimator;
};


#endif
