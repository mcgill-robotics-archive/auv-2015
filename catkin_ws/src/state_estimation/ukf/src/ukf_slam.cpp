#include "ukf.h"
#include "ukf_slam.h"
#include <math.h>
#include <stdio.h>


Vector2d testInput(x,y);


ukf_slam::ukf_slam(Vector2D testInput):estimator(3){

//call constructor on vector of 2n dimensions
//ukf slam will look like ukf pose
//1.cosntructor creates ukf of size 2n
//2.update will take in sensor readings
// observe-=-"h" 

//3.s

}

void ukf_pose::propogate(Eigen::VectorXd rotation, Ref<Eigen::VectorXd> state)
{
}

void ukf_pose::observe()
{
	
}	

void ukf_pose::update()
{
}
