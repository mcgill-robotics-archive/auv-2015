#ifndef UKF_SLAM_
#define UKF_SLAM_

#include <eigen3/Eigen/Dense>
#include "ukf.h"

using namespace Eigen;

//typedef Matrix<double, 3, 6> Matrix3X6d;
//typedef const Ref<const Vector3d> constVector;



class ukf_slam
{
	public:
    	ukf_slam(); 
    	void update();
		//void propogate(Vector3d, Ref<Vector3d>);
		void static propogate(Eigen::VectorXd, Ref<Eigen::VectorXd>);
		//void h(Vector3d, Ref<Vector3d>);
		void static observe(Eigen::VectorXd, Ref<Eigen::VectorXd>);
		
	private:
		ukf estimator;
};


#endif
