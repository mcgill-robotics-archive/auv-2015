#ifndef UKF_POSE_
#define UKF_POSE_

#include <eigen3/Eigen/Dense>
#include "ukf.h"

using namespace Eigen;

typedef Matrix<double, 3, 6> Matrix3X6d;
typedef const Ref<const Vector3d> constVector3;  //Changed to constVector3 to differentiate with constVector (Xd) defined in Xd

class ukf_pose
{
  public:
    ukf_pose(); 
      void update(constVector3 acc, constVector3 rotation, Ref<Vector3d>);
      static void propogate(constVector3, Ref<Vector3d>);
      static MatrixXd observe(MatrixXd);
      void fixState(Ref<Vector3d>);

  private:
    ukf estimator;
  static AngleAxisd angleAxis(Vector3d);
};

#endif
