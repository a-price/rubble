/**
 * \file TestMovementTracker.cpp
 * \brief Tests movement tracking
 *
 * \author Laura Strickland
 * \date October 13, 2013
 *
 * \copyright
 *
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <rubble/MovementTracker.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>



class TestMovementTracker : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestMovementTracker );
	CPPUNIT_TEST(TestMakeSequence);
	CPPUNIT_TEST_SUITE_END();
public:

//	std::vector<gazebo_msgs::ModelStates> states;
	gazebo_msgs::ModelStates states;

	virtual void setUp()
	{
		//Board with zero velocity
//		gazebo_msgs::ModelStates staysPut;
//		rubble::MovementTracker mtStaysPut;

		std::string staysPutName = "staysPut";

		geometry_msgs::Pose PoseA;
		PoseA.position.x = 0.0;
		PoseA.position.y = 0.0;
		PoseA.position.z = 0.0;
		PoseA.orientation.x = 0.0;
		PoseA.orientation.y = 0.0;
		PoseA.orientation.z = 0.0;
		PoseA.orientation.w = 0.0;

		geometry_msgs::Twist TwistA;
		TwistA.linear.x = 0.0;
		TwistA.linear.y = 0.0;
		TwistA.linear.z = 0.0;
		TwistA.angular.x = 0.0;
		TwistA.angular.y = 0.0;
		TwistA.angular.z = 0.0;

//		gazebo_msgs::ModelStates msStaysPut;
		states.name.push_back(staysPutName);
		states.pose.push_back(PoseA);
		states.twist.push_back(TwistA);

//		mtStaysPut.MovementTracker::IntegrateMovement(staysPut);
//		states.push_back(staysPut);

		// Stationary Board
//		gazebo_msgs::ModelStates moves;
//		rubble::MovementTracker mtMoves;

		std::string movesName = "moves";

		geometry_msgs::Pose PoseB;
		PoseB.position.x = 0.0;
		PoseB.position.y = 0.0;
		PoseB.position.z = 0.0;
		PoseB.orientation.x = 0.0;
		PoseB.orientation.y = 0.0;
		PoseB.orientation.z = 0.0;
		PoseB.orientation.w = 0.0;

		geometry_msgs::Twist TwistB;
		TwistB.linear.x = 1.0;
		TwistB.linear.y = 2.0;
		TwistB.linear.z = 3.0;
		TwistB.angular.x = 4.0;
		TwistB.angular.y = 5.0;
		TwistB.angular.z = 6.0;

//		gazebo_msgs::ModelStates msMoves;
		states.name.push_back(movesName);
		states.pose.push_back(PoseB);
		states.twist.push_back(TwistB);

//		mtMoves.MovementTracker::IntegrateMovement(moves);
//		states.push_back(moves);

	}

	virtual void tearDown () {}

	void TestMakeSequence()
	{

		rubble::MovementTracker mt;
		std::vector<double> moved;
		double m;
		std::cout << std::endl;

		moved = mt.IntegrateMovement(states);

		for (int i = 0; i < states.name.size(); i++)
		{
			std::cout << moved[i] << std::endl;
		}
	}
};

CPPUNIT_TEST_SUITE_REGISTRATION(TestMovementTracker);

//int main(int argc, char **argv)
//{
//	::testing::InitGoogleTest(&argc, argv);
//	return RUN_ALL_TESTS();
//}

