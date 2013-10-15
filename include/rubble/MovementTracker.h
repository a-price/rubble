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
	MovementTracker();
	std::vector<double> IntegrateMovement(gazebo_msgs::ModelStates);
private:
	ros::Publisher mstatesPub;
	ros::NodeHandle nh;
};
}
#endif // MOVEMENTTRACKER_H
