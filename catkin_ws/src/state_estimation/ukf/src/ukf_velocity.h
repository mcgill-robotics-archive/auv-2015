#ifndef UKF_VELOCITY_
#define UKF_VELOCITY_

#include <eigen3/Eigen/Dense>
#include "ukf.h"

using namespace Eigen;

class ukf_velocity
{
	public:
		ukf_velocity();
		void update(Vector2d acc, float dt, Ref<Vector2d> outVelocity);
		static void propogate(const Vector2d, float dt, Ref<Vector2d> state);
		static MatrixXd observe(MatrixXd); 

	private:
		ukf estimator;
		MatrixXd processNoise;
		MatrixXd measurementNoise;
};		
#endif
