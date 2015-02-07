#ifndef UKF_POSE_
#define UKF_POSE_

#include <eigen3/Eigen/Dense>
#include "ukf.h"

using namespace Eigen;

typedef Matrix<double, 3, 6> Matrix3X6d;
<<<<<<< HEAD
typedef const Ref<const VectorXd> constVector;
=======
//typedef const Ref<const Vector3d> constVector;
>>>>>>> 85a9d2ffe6a235c9b2c5718c624e9b15c36b1467



class ukf_pose
{
    public:
    	ukf_pose(); 
        void update(constVector acc, constVector rotation, Ref<Vector3d>);
        //void propogate(Vector3d, Ref<Vector3d>);
        static void propogate(Eigen::VectorXd, Ref<Eigen::VectorXd>); //TODO(max) We don't need "Eigen::" since we opened that namespace
        //void h(Vector3d, Ref<Vector3d>);
        static MatrixXd observe(Eigen::MatrixXd);
        void fixState(Ref<Vector3d>);

    private:
        ukf estimator;
	static AngleAxisd angleAxis(Vector3d);
};


#endif
