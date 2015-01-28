#include "ukf.h"
#include "ukf_slam.h"
#include <math.h>
#include <stdio.h>


Vector2d testInput(x,y);

const double pi = std::acos(-1.0);


//initialize matrix which will hold obstacles to be tracked
// n-1 obstacles + 1 for robot (size of Obst should be 2n for x,y positions)

VectorXd state;


//Robots position at state[0], state[1]
//Gate is at position (0,0)
state(2) = 0.0;
state(3) = 0.0;


ukf_slam::ukf_slam(Vector2D testInput):estimator(3){

//call constructor on vector of 2n dimensions
//ukf slam will look like ukf pose
//1.cosntructor creates ukf of size 2n
//2.update will take in sensor readings
// observe-=-"h" 

//3.s

}


void ukf_slam::observe() //Will take id of object later on
{
	

	
}	

void ukf_slam::propogate(Ref<Eigen::VectorXd> state)
{


}


void ukf_slam::update(int Name, Vector2d Dist)
{
	estimator.predict(&propogate);
	estimator.correct(Dist, &observe);

}
