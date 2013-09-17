/**
 * \file random_spawn.cpp
 * \brief Creates a random pile of stuff to play with
 *
 * \author Andrew Price
 * \date September 13, 2013
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


#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#ifndef GAZEBO_MSGS_MESSAGE_SPAWNMODEL_H
#include </home/arprice/gazebo_ros_ws/devel/include/gazebo_msgs/SpawnModel.h>
#endif

const std::string modelName = "boxModel";
const std::string modelString = "<?xml version='1.0'?><sdf version='1.4'><model name=\"my_robot\"><static>false</static><link name='link'><pose>0 0 0 0 0 0</pose><collision name='collision'><geometry><box><size>.4 .2 .1</size></box></geometry></collision><visual name='visual'><geometry><box><size>.4 .2 .1</size></box></geometry></visual></link></model></sdf>";

ros::Publisher zPub;

void stateCallback(const gazebo_msgs::ModelStatesConstPtr states)
{
	int targetIdx = -1;
	for (std::string name : states->name)
	{
		targetIdx++;
		if (name == "testModel")
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
	ros::init(argc, argv, "random_spawn");
	ros::NodeHandle nh;

	zPub = nh.advertise<std_msgs::Float32>("/z",1);
	ros::Subscriber stateSub = nh.subscribe("/gazebo/model_states", 1, &stateCallback);

	ros::ServiceClient deleteClient = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
	gazebo_msgs::DeleteModelRequest dreq;
	gazebo_msgs::DeleteModelResponse dresp;

	dreq.model_name = "testModel";
	deleteClient.call(dreq, dresp);
	ROS_INFO_STREAM("Delete Response:\n" << dresp);


	gazebo_msgs::SpawnModelRequest req;
	gazebo_msgs::SpawnModelResponse resp;

	//ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
	ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");

	req.model_name = nameString;
	req.reference_frame = "map";
	req.model_xml = modelString;

	req.initial_pose.position.x = 0;
	req.initial_pose.position.y = 0;
	req.initial_pose.position.z = 1;

	if (!client.waitForExistence(ros::Duration(2.0)))
	{
		ROS_FATAL_STREAM("Unable to locate service '" << client.getService() << "'");
	}
	client.call(req,resp);

	ROS_INFO_STREAM("Response:\n" << resp);

	ros::spin();

	return 0;
}
