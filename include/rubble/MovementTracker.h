#ifndef MOVEMENTTRACKER_H
#define MOVEMENTTRACKER_H
#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h>

namespace rubble
{
class MovementTracker
{
public:
	MovementTracker(ros::NodeHandle& _nh);
	double IntegrateMovement(gazebo_msgs::ModelStates);

	void ignoreModel(const std::string name);
	void regardModel(const std::string name);
private:
	ros::Publisher mstatesPub;
	ros::NodeHandle nh;

	ros::ServiceClient pauseSrvClient;
	ros::ServiceClient unpauseSrvClient;

	std::set<std::string> ignoreModels;

	double runningAverage;
	const double windowSize;

};
}
#endif // MOVEMENTTRACKER_H
