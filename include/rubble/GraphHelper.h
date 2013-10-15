/**
 * \file rubbleGraphHelper.h
 * \brief
 *
 * \author Andrew Price
 * \date October 7, 2013
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

#ifndef RUBBLEGRAPHHELPER_H
#define RUBBLEGRAPHHELPER_H

#include <map>
#include <rubble/RubbleGraph.h>

namespace rubble
{

typedef std::map<std::string, Graph::vertex_descriptor> VertexMap;

class GraphHelper
{
public:
	GraphHelper();
	~GraphHelper();

	Node::Ptr getNode(std::string nodeID);
	std::set<std::string> getParents(std::string nodeID);
	std::set<std::string> getChildren(std::string nodeID);

	/**
	 * @brief Attempts to add a node to the graph.
	 * If the node already exists, it returns the existing vertex.
	 * @return Reference to either new or existing vertex
	 */
	Node& addNode(std::string);

	bool deleteNode(std::string);

	/**
	 * @brief Attempts to add an edge between the two nodes specified by the strings.
	 * If either node does not exist, it is created.
	 * @return
	 */
	Edge& addEdge(std::string from, std::string to);

	std::set<std::string> getNodeIDs();
	std::set<std::string> getIndependent();
	std::vector<std::pair<unsigned int, std::string> > sortByDegree();

	/**
	 * @brief Creates a graphviz file string for the graph
	 * @return
	 */
	std::string toDot();

protected:
	Graph graph;
	VertexMap vertexMap;
};

} // Rubble
#endif // RUBBLEGRAPHHELPER_H
