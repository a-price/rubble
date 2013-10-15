/**
 * \file GraphHelper.cpp
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

#include "rubble/GraphHelper.h"

#include <sstream>

namespace rubble
{

GraphHelper::GraphHelper()
{
}

GraphHelper::~GraphHelper()
{
}

Node::Ptr GraphHelper::getNode(std::string nodeID)
{
	VertexMap::iterator iter = vertexMap.find(nodeID);
	if (vertexMap.end() != iter)
	{
		Node* node = &(graph[iter->second]);
		return Node::Ptr(node);
	}

	return Node::Ptr(); // Null pointer
}

Node& GraphHelper::addNode(std::string newNode)
{
	// Check our name map for whether the node exists
	VertexMap::iterator iter = vertexMap.find(newNode);
	if (vertexMap.end() != iter)
	{
		Node& node = graph[iter->second];
		return node;
	}

	// Add the node
	Graph::vertex_descriptor v = boost::add_vertex(graph);
	Node& node = graph[v];
	node.debrisName = newNode;

	vertexMap.insert(std::pair<std::string, Graph::vertex_descriptor>(newNode, v));

	return node;
}

Edge& GraphHelper::addEdge(std::string from, std::string to)
{
	// Check if from exists
	VertexMap::iterator iterA = vertexMap.find(from);
	if (vertexMap.end() == iterA)
	{
		addNode(from);
		iterA = vertexMap.find(from);
	}

	// Check if to exists
	VertexMap::iterator iterB = vertexMap.find(to);
	if (vertexMap.end() == iterB)
	{
		addNode(to);
		iterB = vertexMap.find(to);
	}

	// Check if link going the opposite way already exists
	boost::graph_traits<Graph>::out_edge_iterator out_i, out_end;
	for (boost::tie(out_i, out_end) = boost::out_edges(vertexMap.at(to), graph);
		 out_i != out_end;
		 ++out_i)
	{
		Graph::vertex_descriptor toV = boost::target(*out_i, graph);
		Node& toNode = graph[toV];
		if (toNode.debrisName == from) // bidirectional dep is not allowed
		{
			return graph[*out_i];
		}
	}

	// Add the edge
	std::pair<Graph::edge_descriptor, bool> e = boost::add_edge(iterA->second, iterB->second, graph);
	Graph::edge_descriptor eID = e.first;

	return graph[eID];
}

std::set<std::string> GraphHelper::getParents(std::string nodeID)
{
	std::set<std::string> retVal;
	VertexMap::iterator iter = vertexMap.find(nodeID);
	if (vertexMap.end() == iter)
	{
		return retVal;
	}

	boost::graph_traits<Graph>::out_edge_iterator out_i, out_end;

	for (boost::tie(out_i, out_end) = boost::out_edges(iter->second, graph);
		 out_i != out_end;
		 ++out_i)
	{
		Graph::vertex_descriptor toV = boost::target(*out_i, graph);
		Node& toNode = graph[toV];
		retVal.insert(toNode.debrisName);
	}

	return retVal;
}

std::set<std::string> GraphHelper::getChildren(std::string nodeID)
{
	std::set<std::string> retVal;
	VertexMap::iterator iter = vertexMap.find(nodeID);
	if (vertexMap.end() == iter)
	{
		return retVal;
	}

	boost::graph_traits<Graph>::in_edge_iterator in_i, in_end;

	for (boost::tie(in_i, in_end) = boost::in_edges(iter->second, graph);
		 in_i != in_end;
		 ++in_i)
	{
		Graph::vertex_descriptor fromV = boost::target(*in_i, graph);
		Node& toNode = graph[fromV];
		retVal.insert(toNode.debrisName);
	}

	return retVal;
}

std::set<std::string> GraphHelper::getIndependent()
{
	std::set<std::string> retVal;

	for (std::pair<std::string, Graph::vertex_descriptor> entry : vertexMap)
	{
		//std::cout << entry.first << ": " << boost::out_degree(entry.second, graph) << std::endl;
		if (0 == boost::out_degree(entry.second, graph))
		{
			retVal.insert(entry.first);
		}
	}

	return retVal;
}

std::vector<std::pair<unsigned int, std::string> > GraphHelper::sortByDegree()
{
	std::vector<std::pair<unsigned int, std::string> > retVal;

	for (std::pair<std::string, Graph::vertex_descriptor> entry : vertexMap)
	{
		std::pair<unsigned int, std::string> newEntry(boost::out_degree(entry.second, graph), entry.first);
		retVal.push_back(newEntry);
	}

	std::sort(retVal.begin(), retVal.end());

	return retVal;
}

std::set<std::string> GraphHelper::getNodeIDs()
{
	std::set<std::string> retVal;
	for (std::pair<std::string, Graph::vertex_descriptor> entry : vertexMap)
	{
		retVal.insert(entry.first);
	}
	return retVal;
}

std::string GraphHelper::toDot()
{
	std::ostringstream outfile;
	outfile << "digraph G {\n";
	outfile << "rankdir=\"BT\";\n";

	// Loop through all edges
	Graph::edge_iterator edgeIt, edgeEnd;

	for (boost::tie(edgeIt, edgeEnd) = boost::edges(graph);
		 edgeIt != edgeEnd;
		 ++edgeIt)
	{
		Graph::vertex_descriptor fromV = boost::source(*edgeIt, graph);
		Graph::vertex_descriptor toV = boost::target(*edgeIt, graph);
		Node& fromNode = graph[fromV];
		Node& toNode = graph[toV];

		outfile << fromNode.debrisName << "->" << toNode.debrisName << ";\n";
	}

	outfile << "}";

	return outfile.str();
}

} // Rubble
