#include "rubble/MovementTracker.h"
#include <ros/package.h>
#include <ros/service_client.h>
#include <std_srvs/Empty.h>

namespace rubble
{

	MovementTracker::MovementTracker(ros::NodeHandle& _nh)
		: nh(_nh)
	{
	}



	std::vector<double> MovementTracker::IntegrateMovement(gazebo_msgs::ModelStates states)
	{

		mstatesPub = nh.advertise<std_msgs::Float32>("/movementTracker", 1);
		// Make physics pausable/unpausable
		ros::ServiceClient pauseSrvClient = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
		std_srvs::EmptyRequest eReq;
		std_srvs::EmptyResponse eResp;
		ros::ServiceClient unpauseSrvClient = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

		std::vector<double> allmoves;
		double totalMove = 0;
		float sumAllMoveF = 0;

		std_msgs::Float32_<float> sumAllMove;

		for (int i = 0; i < states.name.size(); i++)
		{
			geometry_msgs::Twist t = states.twist[i];
			// Add up abs values of twists
			totalMove += t.linear.x;
			totalMove += t.linear.y;
			totalMove += t.linear.z;
			totalMove += (t.angular.x)*0.01;//multiply by scaling factor (this is in radians); using scaling factor of 1 cm
			totalMove += (t.angular.y)*0.01;
			totalMove += (t.angular.z)*0.01;
			allmoves.push_back(totalMove);
		}

		while(!allmoves.empty())
		{
			sumAllMoveF += allmoves.back();
			allmoves.pop_back();
		}
		sumAllMove = sumAllMoveF;

		if(sumAllMoveF < 0.05)// if sum of all movements of all boards is less than 5cm. This value may need adjustment.
		{
			mstatesPub.publish(sumAllMove); //if movement below threshold, publish the resulting really small sum
			pauseSrvClient.call(eReq, eResp); // pause physics
			ros::Duration(5.0).sleep(); // sleep for 5 seconds (add other stuff to do here instead?)
			unpauseSrvClient.call(eReq, eResp); // unpause physics

		}

		return allmoves;
	}
}// namespace rubble
