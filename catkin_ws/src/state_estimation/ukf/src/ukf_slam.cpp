#include "ukf.h"
#include "ukf_slam.h"
#include <math.h>
#include <stdio.h>


Vector2d testInput(x,y);

const double pi = std::acos(-1.0);


//initialize matrix which will hold obstacles to be tracked
// n-1 obstacles + 1 for robot (size of Obst should be 2n for x,y positions)

//double Obst[]

//Robots position at Obst[0], Obst[1]
//Gate is at position (0,0)
Obst[2] = 0.0;
Obst[3] = 0.0;





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
