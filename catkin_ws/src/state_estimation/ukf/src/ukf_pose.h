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
        void update(constVector acc, constVector rotation, Ref<Vector3d>);
        //void propogate(Vector3d, Ref<Vector3d>);
        static void propogate(Eigen::VectorXd, Ref<Eigen::VectorXd>); //TODO(max) We don't need "Eigen::" since we opened that namespace
        //void h(Vector3d, Ref<Vector3d>);
        static void observe(Eigen::VectorXd, Ref<Eigen::VectorXd>);
        void fixState(Ref<Vector3d>);

    private:
        ukf estimator;
	static AngleAxisd angleAxis(Vector3d);
};


#endif
