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

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>

#include "rubble/GraphHelper.h"

/**
 * @brief Motion below which is considered a static scene
 */
const double STABLE_THRESHOLD = 0.8;

/**
 * @brief Motion above which is considered a failed move
 */
const double UNSTABLE_THRESHOLD = 10.0;

ros::NodeHandlePtr nh;
ros::Subscriber contactSub;
ros::Subscriber stateSub;
ros::Subscriber motionSub;
ros::Publisher ignorePub;
ros::Publisher regardPub;
ros::ServiceClient deleteClient;
ros::ServiceClient pauseSrvClient;
ros::ServiceClient unpauseSrvClient;
ros::ServiceClient resetClient;
rubble::GraphHelper graph;
ros::Time tLocked;
ros::Duration tWait;
bool isGatheringContactInfo = true;
bool isUninitialized = true;

gazebo_msgs::ModelStates currentState;
std::deque<gazebo_msgs::ModelStates> stateLog;
std::deque<std::string> ignoredModels; // Models that have been "deleted"

// Monitor what's worked and what hasn't
int removalDepth = 0;
std::vector<std::set<std::string> > failedModels;

// Forward declarations for functions
std::string proposeRemoval();
void logMove();
void removeItem(std::string& id);
void undoMove();

std::string replace(const std::string& str, const std::string& from, const std::string& to)
{
	std::string copy = str;
	size_t start_pos = str.find(from);
	if(start_pos == std::string::npos)
		return copy;
	copy.replace(start_pos, from.length(), to);
	return copy;
}

void rebuildContactGraph()
{
	// Delete the contact graph
	graph = rubble::GraphHelper();
	tLocked = ros::Time::now();
	isGatheringContactInfo = true;
}

std::string proposeRemoval()
{
	std::cerr << "Proposing...";

	std::vector<std::pair<unsigned int, std::string> > order = graph.sortByDegree();
	for (int i = 0; i < order.size(); i++)
	{
		std::pair<unsigned int, std::string>& item = order[i];
		std::cout << item.second << ": " << item.first << std::endl;
	}

	// return the most likely node to be freeable
	std::vector<std::pair<unsigned int, std::string> > suggestions = graph.sortByDegree();
	for (std::vector<std::pair<unsigned int, std::string> >::iterator iter = suggestions.begin();
		 iter != suggestions.end();
		 iter++)
	{
		// Check if in failed list for current depth
		std::set<std::string>::iterator alreadyFailed = failedModels[removalDepth].find(iter->second);
		if (alreadyFailed == failedModels[removalDepth].end())
		{
			return iter->second;
		}
	}

	// All remaining blocks have been tried and have failed, go up a level.
	undoMove();

	return "";
}

void removeItem(std::string& id)
{
	std::cerr << "Deleting" << std::endl;
	// Pause physics
	std_srvs::EmptyRequest eReq;
	std_srvs::EmptyResponse eResp;
	pauseSrvClient.call(eReq, eResp);

	// Delete the model from monitoring
	ignoredModels.push_back(id);
	graph.deleteNode(id);

	// Move the model out of the main sim area
	gazebo_msgs::SetModelStateRequest req;
	gazebo_msgs::SetModelStateResponse resp;
	req.model_state.model_name = id;
	req.model_state.pose.position.x = 5;
	req.model_state.pose.position.y = 1;
	req.model_state.pose.position.z = 2;
	req.model_state.pose.orientation.w = 1;
	req.model_state.pose.orientation.x = 0;
	req.model_state.pose.orientation.y = 0;
	req.model_state.pose.orientation.z = 0;
	resetClient.call(req, resp);

	// One more item removed
	removalDepth++;

	// Un-track motion
	std_msgs::String msg;
	msg.data = id;
	ignorePub.publish(msg);

	// Rebuild the contact graph
	rebuildContactGraph();

	// Re-enable Physics
	unpauseSrvClient.call(eReq, eResp);
}

void logMove()
{
	std::cerr << "Logging" << std::endl;
	stateLog.push_back(currentState);
}

void undoMove()
{
	if (removalDepth < 1)
	{
		std::cerr << "Already at root removal." << std::endl;
		return;
	}
	std::cerr << "Reverting" << std::endl;
	// Respawn world to checkpoint
	gazebo_msgs::ModelStates prevStates = stateLog.at(stateLog.size()-1);
	stateLog.pop_back();

	for (int i = 0; i < prevStates.name.size(); i++)
	{
		gazebo_msgs::SetModelStateRequest req;
		gazebo_msgs::SetModelStateResponse resp;

		req.model_state.model_name = prevStates.name[i];
		req.model_state.pose = prevStates.pose[i];
		req.model_state.twist = prevStates.twist[i];

		resetClient.call(req, resp);
	}

	// One less item removed
	removalDepth--;

	std::string attempted = ignoredModels.at(ignoredModels.size()-1);

	// Mark as failed
	failedModels[removalDepth].insert(attempted);

	// Re-track motion
	std_msgs::String msg;
	msg.data = attempted;
	regardPub.publish(msg);

	// Restore the item's standing
	ignoredModels.pop_back();

	// Rebuild the contact graph
	rebuildContactGraph();
}

void contactCallback(const gazebo_msgs::ContactsStateConstPtr contacts)
{
	if (!isGatheringContactInfo) { return; }
	if (0 == contacts->states.size()) {	return;	}

	// Find names of all debris pieces touching
	for (int i = 0; i < contacts->states.size(); i++)
	{
		const gazebo_msgs::ContactState contact = contacts->states[i];

		if (contact.collision1_name == "" || contact.collision2_name == "")
		{
			std::cerr << "WTF?" << std::endl;
			continue;
		}

		std::string a = replace(contact.collision1_name, "::link::collision", "");
		std::string b = replace(contact.collision2_name, "::link::collision", "");

		bool isIgnored = false;
		// Don't add elements to the map that are "deleted"
		for (std::deque<std::string>::iterator iter = ignoredModels.begin();
			 iter != ignoredModels.end();
			 ++iter)
		{
			if (a == *iter || b == *iter)
			{
				isIgnored = true;
				break;
			}
		}

		if (isIgnored) {continue;}

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
		isGatheringContactInfo = false;
		std::ofstream os;
		os.open("/home/arprice/graphs/graph" + std::to_string(removalDepth) + ".dot");
		os << graph.toDot();
		os.close();
	}
}

void stateCallback(const gazebo_msgs::ModelStatesConstPtr states)
{
	currentState = *states;
	if (isUninitialized)
	{
		failedModels.resize(states->name.size());
		isUninitialized = false;
	}
}

void movementCallback(const std_msgs::Float32ConstPtr motion)
{
	if (isGatheringContactInfo) { return; }

	if (motion->data < STABLE_THRESHOLD)
	{
		std::string nextPiece = proposeRemoval();
		if (nextPiece == "")
		{
			return;
		}
		std::cerr << nextPiece << std::endl;
		logMove();
		removeItem(nextPiece);
	}
	else if (motion->data > UNSTABLE_THRESHOLD)
	{
		undoMove();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "removal_planner");
	nh.reset(new ros::NodeHandle);

	sleep(15);

	tLocked = ros::Time::now();
	tWait = ros::Duration(1, 0);
	isGatheringContactInfo = true;

	deleteClient = nh->serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
	contactSub = nh->subscribe("/gazebo/contacts", 1, &contactCallback);
	stateSub = nh->subscribe("/gazebo/model_states", 1, &stateCallback);
	motionSub = nh->subscribe("/motion_tracker/net_motion", 1, &movementCallback);
	ignorePub = nh->advertise<std_msgs::String>("/motion_tracker/ignore_model", 1);
	regardPub = nh->advertise<std_msgs::String>("/motion_tracker/regard_model", 1);
	pauseSrvClient = nh->serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
	unpauseSrvClient = nh->serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
	resetClient = nh->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

	ros::spin();

	return 0;
}
