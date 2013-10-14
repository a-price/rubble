#include "rubble/MovementTracker.h"

namespace rubble
{

	MovementTracker::MovementTracker()
	{
	}



	std::vector<double> MovementTracker::IntegrateMovement(gazebo_msgs::ModelStates states)
	{
		std::vector<double> allmoves;
		double totalMove = 0;
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
		return allmoves;
	}
}// namespace rubble
