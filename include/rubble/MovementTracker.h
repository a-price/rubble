#ifndef MOVEMENTTRACKER_H
#define MOVEMENTTRACKER_H
#include <gazebo_msgs/ModelStates.h>

namespace rubble
{
class MovementTracker
{
public:
	MovementTracker();
	std::vector<double> IntegrateMovement(gazebo_msgs::ModelStates);

};
}
#endif // MOVEMENTTRACKER_H
