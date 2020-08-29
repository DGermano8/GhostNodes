/* Generated file, do not edit */

#ifndef CXXTEST_RUNNING
#define CXXTEST_RUNNING
#endif

#define _CXXTEST_HAVE_STD
#include <cxxtest/TestListener.h>
#include <cxxtest/TestTracker.h>
#include <cxxtest/TestRunner.h>
#include <cxxtest/RealDescriptions.h>
#include <cxxtest/ErrorPrinter.h>

#include "CommandLineArguments.hpp"
int main( int argc, char *argv[] ) {
 CommandLineArguments::Instance()->p_argc = &argc;
 CommandLineArguments::Instance()->p_argv = &argv;
 return CxxTest::ErrorPrinter().run();
}
#include "projects/GhostNodes/test/TumorGrowthWithConfluence/TestContactInhibitionGhosts.hpp"

static TestRunningContactInhibitionSimulationsTutorial suite_TestRunningContactInhibitionSimulationsTutorial;

static CxxTest::List Tests_TestRunningContactInhibitionSimulationsTutorial = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_TestRunningContactInhibitionSimulationsTutorial( "projects/GhostNodes/test/TumorGrowthWithConfluence/TestContactInhibitionGhosts.hpp", 247, "TestRunningContactInhibitionSimulationsTutorial", suite_TestRunningContactInhibitionSimulationsTutorial, Tests_TestRunningContactInhibitionSimulationsTutorial );

static class TestDescription_TestRunningContactInhibitionSimulationsTutorial_TestContactInhibitionInBox : public CxxTest::RealTestDescription {
public:
 TestDescription_TestRunningContactInhibitionSimulationsTutorial_TestContactInhibitionInBox() : CxxTest::RealTestDescription( Tests_TestRunningContactInhibitionSimulationsTutorial, suiteDescription_TestRunningContactInhibitionSimulationsTutorial, 251, "TestContactInhibitionInBox" ) {}
 void runTest() { suite_TestRunningContactInhibitionSimulationsTutorial.TestContactInhibitionInBox(); }
} testDescription_TestRunningContactInhibitionSimulationsTutorial_TestContactInhibitionInBox;

#include <cxxtest/Root.cpp>
