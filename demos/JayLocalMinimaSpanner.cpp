#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/multilevel/planners/multimodal/LocalMinimaSpanners.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/config.h>
#include <iostream>
#include <thread>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace om = ompl::multilevel;

/*
This is a 2D solver example that finds multiple solutions to the goal.
*/

bool circ(float x0, float y0, float r, const ob::RealVectorStateSpace::StateType &pos)
{
    return (((pos[0] - x0) * (pos[0] - x0) + (pos[1] - y0) * (pos[1] - y0)) > r * r);
}

bool isStateValid(const ob::State *state)
{
    const ob::RealVectorStateSpace::StateType &pos = *state->as<ob::RealVectorStateSpace::StateType>();
    
    return (circ(-0.5, 0, 0.45, pos));
}

void plan()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    // set the bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);

    //create instance of space information
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);

    // create a start state
    ob::ScopedState<> start(space);
    start[0] = -1;
    start[1] = 0;

    // create a goal state
    ob::ScopedState<> goal(space);
    goal[0] = 1;
    goal[1] = 0;

    // create an instance of problem definition
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Choose the planner
    std::cout << "\nUsing Local Minima Spanner:" << std::endl;
    std::vector<ob::SpaceInformationPtr> siVec; siVec.push_back(si);
    auto planner = std::make_shared<om::LocalMinimaSpanners>(siVec);
    planner->setProblemDefinition(pdef);
    planner->setup();
    // ss.setPlanner(planner);
    // auto pathSpacePlanner = std::dynamic_pointer_cast<ompl::multilevel::LocalMinimaSpanners>(planner);
    // auto localMinimaTree = pathSpacePlanner->getLocalMinimaTree();
    // std::cout << localMinimaTree->getNumberOfMinima() << std::endl;

    // attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        pdef->getSolutionPath()->print(std::cout);

        // std::ofstream out;
        // out.open("/home/jay/TU_Berlin_Thesis/Example_out.txt", std::ios_base::app);
        // ss.getSolutionPath().printAsMatrix(out);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    // std::ofstream out("/home/jay/TU_Berlin_Thesis/Example_out.txt");  // create an empty txt file
    plan();

    return 0;
}