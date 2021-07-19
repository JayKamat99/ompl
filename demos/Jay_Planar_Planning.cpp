#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <thread>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/*
This is a 2D solver example problem that has circles as obstacles.
*/

bool isStateValid(const ob::State *state)
{
    const ob::RealVectorStateSpace::StateType& pos = *state->as<ob::RealVectorStateSpace::StateType>();
    // Valid states satisfy the following constraints:
    // -1<= x,y,z <=1
    // if .25 <= z <= .5, then |x|>.8 and |y|>.8
    // return !(fabs(pos[0])<.8 && fabs(pos[1])<.8 && pos[2]>.25 && pos[2]<.5);
    int r = 0.5;
    return (pos[0]*pos[0] + pos[1]*pos[1] > r*r);
}

void plan(int plannerIndex)
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    // set the bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker(isStateValid);

    // create a start state
    ob::ScopedState<> start(space);
    start[0] = -1;
    start[1] = 0;

    // create a goal state
    ob::ScopedState<> goal(space);
    goal[0] = 1;
    goal[1] = 0;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    //Choose the planner else default is ________.
    if (plannerIndex == 0)
    	auto planner(std::make_shared<og::SPARS>(ss.getSpaceInformation()));
    else if (plannerIndex == 1)
    	auto planner(std::make_shared<og::PRM>(ss.getSpaceInformation()));
    else if (plannerIndex == 2)
    	auto planner(std::make_shared<og::RRT>(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = ss.solve();
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        std::ofstream out;
        out.open("/home/jay/TU_Berlin_Thesis/Example_out.txt", std::ios_base::app);
        // out.open("/home/jay/TU_Berlin_Thesis/PRM_out.txt");
        ss.getSolutionPath().printAsMatrix(out);
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::ofstream out("/home/jay/TU_Berlin_Thesis/Example_out.txt");
    std::cout << "Using RRT algorithm:" << std::endl;
    plan(0);
    std::cout << "\nUsing PRM algorithm:" << std::endl;
    plan(1);
    std::cout << "\nUsing SPARS roadmap algorithm:" << std::endl;
    plan(2);

    return 0;
}