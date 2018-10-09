#define BOOST_TEST_MODULE "ExcaliburFrameProcessorTests"
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <boost/shared_ptr.hpp>

#include <iostream>

#include "ExcaliburProcessPlugin.h"
#include "DebugLevelLogger.h"

IMPLEMENT_DEBUG_LEVEL;

class ExcaliburProcessPluginTestFixture
{
public:
	ExcaliburProcessPluginTestFixture()
	{
		std::cout << "ExcaliburProcessPluginTestFixture constructor" << std::endl;
	}

	~ExcaliburProcessPluginTestFixture()
	{
		std::cout << "ExcaliburProcessPluginTestFixture destructor" << std::endl;
	}
};

BOOST_FIXTURE_TEST_SUITE(ExcaliburProcessPluginUnitTest, ExcaliburProcessPluginTestFixture);

BOOST_AUTO_TEST_CASE(ExcaliburProcessPluginTestFixture)
{
	std::cout << "ExcaliburProcessPluginTestFixture test case" << std::endl;
}

BOOST_AUTO_TEST_SUITE_END();
