/**
 * \file TestGraphFollow.cpp
 * \brief Verifies operation of ParameterizedObject classes
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

//#include "gtest/gtest.h"
#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "rubble/GraphHelper.h"

//using namespace hubo_motion_ros;

class TestGraphFollow : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestGraphFollow );
	CPPUNIT_TEST(TestMakeSequence);
	CPPUNIT_TEST_SUITE_END();
public:

	std::vector<rubble::GraphHelper> graphs;
	virtual void setUp()
	{
		// Simple chain
		rubble::GraphHelper graphChain;
		graphChain.addEdge("B", "A");
		graphChain.addEdge("C", "B");
		graphs.push_back(graphChain);

		// Simple tree
		rubble::GraphHelper graphTree;
		graphTree.addEdge("B", "A");
		graphTree.addEdge("C", "B");
		graphTree.addEdge("D", "B");
		graphs.push_back(graphTree);

		// Simple cycle
		rubble::GraphHelper graphCycle;
		graphCycle.addEdge("B", "A");
		graphCycle.addEdge("C", "B");
		graphCycle.addEdge("A", "C");
		graphs.push_back(graphCycle);

	}

	virtual void tearDown () {}

	void TestMakeSequence()
	{
		for (rubble::GraphHelper graph : graphs)
		{
			std::set<std::string> free = graph.getIndependent();
			for (std::set<std::string>::iterator i = free.begin();
				 i != free.end();
				 ++i)
			{
				std::cout << *i << std::endl;
			}
		}
	}
};

CPPUNIT_TEST_SUITE_REGISTRATION(TestGraphFollow);

//int main(int argc, char **argv)
//{
//	::testing::InitGoogleTest(&argc, argv);
//	return RUN_ALL_TESTS();
//}
