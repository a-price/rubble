/**
 * \file example_spawn.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 10 21, 2013
 *
 * \copyright
 *
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab Georgia Institute of Technology
 * Director: Mike Stilman http://www.golems.org
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

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#ifndef GAZEBO_MSGS_MESSAGE_SPAWNMODEL_H
#include </home/arprice/gazebo_ros_ws/devel/include/gazebo_msgs/SpawnModel.h>
#endif

const float dx = 1.0;
const float dy = 0.0381;
const float dz = 0.0889;

const std::string modelName = "boxModel";
//const std::string modelString = "<?xml version='1.0'?><sdf version='1.4'><model name=\"my_robot\"><static>false</static><link name='link'><pose>0 0 0 0 0 0</pose><collision name='collision'><geometry><box><size>1 .1 .05</size></box></geometry></collision><visual name='visual'><geometry><box><size>1 .1 .05</size></box></geometry></visual></link></model></sdf>";

std::vector<std::string> objectNames;

ros::Publisher zPub;

ros::ServiceClient worldInfoClient;
ros::ServiceClient spawnClient;
ros::ServiceClient deleteClient;

float randbetween(float min, float max)
{
	return (max - min) * ( (double)rand() / (double)RAND_MAX ) + min;
}

std::string loadModelSDF(const std::string file)
{
	std::string filename;
	try
	{
		if (file.find("package://") == 0)
		{
			filename = file;
			filename.erase(0, strlen("package://"));
			size_t pos = filename.find("/");
			if (pos != std::string::npos)
			{
				std::string package = filename.substr(0, pos);
				filename.erase(0, pos);
				std::string package_path = ros::package::getPath(package);
				filename = package_path + filename;
			}
		}
		else
		{
			ROS_ERROR("Failed to locate file: %s", file.c_str());
			return "";
		}
	}
	catch (std::exception& e)
	{
		ROS_ERROR("Failed to retrieve file: %s", e.what());
		return "";
	}

	std::ifstream t(filename);
	std::string outstr;

	t.seekg(0, std::ios::end);
	outstr.reserve(t.tellg());
	t.seekg(0, std::ios::beg);

	outstr.assign((std::istreambuf_iterator<char>(t)),
				   std::istreambuf_iterator<char>());

	return outstr;
}

void cleanWorld()
{
	gazebo_msgs::GetWorldPropertiesRequest preq;
	gazebo_msgs::GetWorldPropertiesResponse presp;
	worldInfoClient.call(preq, presp);

	objectNames = presp.model_names;

	for (std::string name : objectNames)
	{
		std::size_t found = name.find(modelName);
		if (found != std::string::npos)
		{
			// Delete the model.
			gazebo_msgs::DeleteModelRequest dreq;
			gazebo_msgs::DeleteModelResponse dresp;

			dreq.model_name = name;
			deleteClient.call(dreq, dresp);
		}
	}
}

void stateCallback(const gazebo_msgs::ModelStatesConstPtr states)
{
	int targetIdx = -1;

	for (std::string name : states->name)
	{
		targetIdx++;
		if (name == modelName)
		{
			break;
		}
	}
	std_msgs::Float32 z;
	z.data = states->pose[targetIdx].position.z;
	zPub.publish(z);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "example_spawn");
	ros::NodeHandle nh;

	srand (time(NULL));

	zPub = nh.advertise<std_msgs::Float32>("/z",1);
	ros::Subscriber stateSub = nh.subscribe("/gazebo/model_states", 1, &stateCallback);

	worldInfoClient = nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
	deleteClient = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
	spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");

	const std::string modelString = loadModelSDF("package://rubble/resources/rubble.sdf");

	cleanWorld();

	if (!spawnClient.waitForExistence(ros::Duration(2.0)))
	{
		ROS_FATAL_STREAM("Unable to locate service '" << spawnClient.getService() << "'");
	}

	std::vector<geometry_msgs::Pose> poses;
	geometry_msgs::Pose pose;
	/*
	pose.orientation.w = 1;
	pose.position.y = 0.3; pose.position.z = dz/2.0;
	poses.push_back(pose);
	pose.position.y = -0.3; pose.position.z = dz/2.0;
	poses.push_back(pose);
	pose.position.y = 0.3; pose.position.z = dz*3.0/2.0;
	poses.push_back(pose);
	pose.position.y = -0.3; pose.position.z = dz*3.0/2.0;
	poses.push_back(pose);
	pose.position.y = 0.0; pose.position.z = dz*5.0/2.0;
	pose.orientation.z = 1;
	poses.push_back(pose);
	pose.position.x = 0.3; pose.position.z = dz*7.0/2.0;
	pose.orientation.z = 0;
	poses.push_back(pose);
	pose.position.x = 1.0; pose.position.z = dz*9.0/2.0;
	poses.push_back(pose);
	*/

	pose.orientation.w = 1;
	pose.position.z = dz/2.0;
	poses.push_back(pose);
	pose.position.z = dz*3.0/2.0;
	poses.push_back(pose);
	pose.orientation.z = 1;
	pose.position.z = dz*5.0/2.0;
	poses.push_back(pose);
	pose.orientation.z = 0;
	pose.position.y = -0.3; pose.position.z = dz*7.0/2.0;
	poses.push_back(pose);
	pose.position.y = 0.3;  pose.position.z = dz*7.0/2.0;
	poses.push_back(pose);

	// Pause physics
	ros::ServiceClient pauseSrvClient = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
	std_srvs::EmptyRequest eReq;
	std_srvs::EmptyResponse eResp;
	pauseSrvClient.call(eReq, eResp);

	for (int i = 0; i < poses.size(); i++)
	{
		gazebo_msgs::SpawnModelRequest req;
		gazebo_msgs::SpawnModelResponse resp;

		req.model_name = modelName + std::to_string(i);
		req.reference_frame = "map";
		req.model_xml = modelString;

		req.initial_pose = poses[i];

		spawnClient.call(req,resp);

		ROS_INFO_STREAM("Response:\n" << resp);
	}

	// Unpause Physics
	ros::ServiceClient unpauseSrvClient = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
	unpauseSrvClient.call(eReq, eResp);

	ros::spinOnce();

	return 0;
}
