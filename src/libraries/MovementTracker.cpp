#include "rubble/MovementTracker.h"
#include <ros/package.h>
#include <ros/service_client.h>
#include <std_srvs/Empty.h>

namespace rubble
{

	MovementTracker::MovementTracker(ros::NodeHandle& _nh)
		: nh(_nh)
	{
		ignoreModels.insert("ground_plane");

		mstatesPub = nh.advertise<std_msgs::Float32>("/motion_tracker/net_motion", 1);

		// Make physics pausable/unpausable
		pauseSrvClient = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
		unpauseSrvClient = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
	}


	double MovementTracker::IntegrateMovement(gazebo_msgs::ModelStates states)
	{
		std_srvs::EmptyRequest eReq;
		std_srvs::EmptyResponse eResp;

		double totalMove = 0;
		bool someModelExists = false;

		std_msgs::Float32 sumAllMove;

		for (int i = 0; i < states.name.size(); i++)
		{
			std::set<std::string>::iterator iter = ignoreModels.find(states.name[i]);
			if (iter != ignoreModels.end())
			{
				continue;
			}
			else
			{
				someModelExists = true;
			}

			geometry_msgs::Twist t = states.twist[i];
			// Add up abs values of twists
			totalMove += fabs(t.linear.x);
			totalMove += fabs(t.linear.y);
			totalMove += fabs(t.linear.z);
			totalMove += fabs(t.angular.x)*0.01;//multiply by scaling factor (this is in radians); using scaling factor of 1 cm
			totalMove += fabs(t.angular.y)*0.01;
			totalMove += fabs(t.angular.z)*0.01;
		}

		sumAllMove.data = totalMove;

		if (someModelExists)
//		if(sumAllMove.data < 0.05)// if sum of all movements of all boards is less than 5cm. This value may need adjustment.
		{
			mstatesPub.publish(sumAllMove); //if movement below threshold, publish the resulting really small sum
//			pauseSrvClient.call(eReq, eResp); // pause physics
//			ros::Duration(5.0).sleep(); // sleep for 5 seconds (add other stuff to do here instead?)
//			unpauseSrvClient.call(eReq, eResp); // unpause physics

		}

		return totalMove;
	}

	void MovementTracker::ignoreModel(const std::string name)
	{
		ignoreModels.insert(name);
	}

	void MovementTracker::regardModel(const std::string name)
	{
		ignoreModels.erase(name);
	}

}// namespace rubble
