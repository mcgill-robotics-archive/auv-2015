#include <gtest/gtest.h>
#include "../include/robot.h"
#include <ros/package.h>
#include <stdlib.h>

/**
 * Test the function calculateDragTorque() from class Robot
 */
TEST(RobotTest, testCalculateDragTorque)
{
	// valid test cases
	
	/*math::Vector3 inputVector;
	math::Vector3 expectedResult;
	geometry_msgs::Vector3 result;
	
	inputVector.x = 10;
	inputVector.y = 10;
	inputVector.z = 10;
	
	expectedResult.x = expectedResult.y = expectedResult.z = -100;
	*/
	//gazebo::Robot r;
	
	/*result = r->calculateDragTorque(inputVector);
	
	result.x = result.y = result.z = 0;
	
	ASSERT_EQ(expectedResult.x, result.x) << "x component of drag torque should be " << expectedResult.x;
	ASSERT_EQ(expectedResult.y, result.y) << "y component of drag torque should be " << expectedResult.y;
	ASSERT_EQ(expectedResult.z, result.z) << "z component of drag torque should be " << expectedResult.z;
	*/
	// invalid test cases
}

int main(int argc, char ** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
