/**
 * \file random_spawn.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 9 13, 2013
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
#include <gazebo_msgs/SpawnModel.h>
#ifndef GAZEBO_MSGS_MESSAGE_SPAWNMODEL_H
#include </home/arprice/gazebo_ros_ws/devel/include/gazebo_msgs/SpawnModel.h>
#endif
int main(int argc, char** argv)
{
	ros::init(argc, argv, "random_spawn");
	ros::NodeHandle nh;

	gazebo_msgs::SpawnModelRequest req;
	gazebo_msgs::SpawnModelResponse resp;

	ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

	req.model_name = "testModel";
	req.reference_frame = "map";
	req.model_xml = "<?xml version=\"1.0\"?><robot name=\"myfirst\"><link name=\"base_link\"><visual><geometry><cylinder length=\"0.6\" radius=\"0.2\"/></geometry></visual></link></robot>";

	req.initial_pose.position.x = 0;
	req.initial_pose.position.y = 0;
	req.initial_pose.position.z = 1;


	client.call(req,resp);

	ROS_INFO_STREAM("Response:\n" << resp);

	return 0;
}
