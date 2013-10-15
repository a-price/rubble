/**
 * \file removal_planner.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 10 7, 2013
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


#include <string>
#include <fstream>
#include <streambuf>
#include <deque>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>

#include "rubble/GraphHelper.h"

/**
 * @brief Motion below which is considered a static scene
 */
const double STABLE_THRESHOLD = 0.1;

/**
 * @brief Motion above which is considered a failed move
 */
const double UNSTABLE_THRESHOLD = 1.0;

ros::NodeHandlePtr nh;
ros::Subscriber contactSub;
ros::Subscriber stateSub;
ros::ServiceClient deleteClient;
ros::ServiceClient pauseSrvClient;
ros::ServiceClient unpauseSrvClient;
rubble::GraphHelper graph;
ros::Time tLocked;
ros::Duration tWait;
bool stateIsLocked = false;
bool trackMotion = false;
gazebo_msgs::ModelStates currentState;
std::deque<gazebo_msgs::ModelStates> stateLog;
std::deque<std::string> ignoredModels; // Models that have been "deleted"

std::string replace(const std::string& str, const std::string& from, const std::string& to)
{
	std::string copy = str;
	size_t start_pos = str.find(from);
	if(start_pos == std::string::npos)
		return copy;
	copy.replace(start_pos, from.length(), to);
	return copy;
}

std::string proposeRemoval()
{
	// return the most likely node to be freeable
	return graph.sortByDegree().begin()->second;
}

void removeItem(std::string& id)
{
	// Pause physics
	std_srvs::EmptyRequest eReq;
	std_srvs::EmptyResponse eResp;
	pauseSrvClient.call(eReq, eResp);

	// Delete the model.
	ignoredModels.push_back(id);
	// TODO: Delete from GraphHelper

	// Re-enable Physics
	unpauseSrvClient.call(eReq, eResp);
}

void logMove()
{
	stateLog.push_back(currentState);
}

void undoMove()
{
	// Respawn world to checkpoint
	gazebo_msgs::ModelStates prevStates = stateLog.at(stateLog.size()-1);
	stateLog.pop_back();
}

void contactCallback(const gazebo_msgs::ContactsStateConstPtr contacts)
{
	if (stateIsLocked) { return; }
	if (0 == contacts->states.size()) {	return;	}

	// Find names of all debris pieces touching
	for (int i = 0; i < contacts->states.size(); i++)
	{
		const gazebo_msgs::ContactState contact = contacts->states[i];

		std::string a = replace(contact.collision1_name, "::link::collision", "");
		std::string b = replace(contact.collision2_name, "::link::collision", "");

		// Don't add elements to the map that are "deleted"
		for (std::deque<std::string>::iterator iter = ignoredModels.begin();
			 iter != ignoredModels.end();
			 ++iter)
		{
			if (a == *iter || b == *iter)
			{
				continue;
			}
		}

		// Add the edge in the correct direction
		if (contact.info == contact.collision1_name)
		{
			graph.addEdge(a, b);
		}
		else
		{
			graph.addEdge(b, a);
		}
	}

	if (ros::Time::now() > tLocked + tWait)
	{
		std::cerr << graph.toDot();
		ros::shutdown();
	}
}

void stateCallback(const gazebo_msgs::ModelStatesConstPtr states)
{
	currentState = *states;
	if (!trackMotion) { return; }

}

void movementCallback(const std_msgs::Float64ConstPtr motion)
{
	if (motion->data < STABLE_THRESHOLD)
	{
		logMove();
		std::string nextPiece = proposeRemoval();
		removeItem(nextPiece);
	}
	else if (motion->data > UNSTABLE_THRESHOLD)
	{
		undoMove();
		// Mark that attempt as failed somehow
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "removal_planner");
	nh.reset(new ros::NodeHandle);

	sleep(15);

	tLocked = ros::Time::now();
	tWait = ros::Duration(5, 0);

	deleteClient = nh->serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
	contactSub = nh->subscribe("/gazebo/contacts", 1, &contactCallback);
	stateSub = nh->subscribe("/gazebo/model_states", 1, &stateCallback);
	pauseSrvClient = nh->serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
	unpauseSrvClient = nh->serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

	ros::spin();

	return 0;
}
