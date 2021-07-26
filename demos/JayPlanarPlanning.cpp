#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
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

// bool circ(float x0, float y0, float r, const ob::RealVectorStateSpace::StateType& pos);

bool circ(float x0, float y0, float r, const ob::RealVectorStateSpace::StateType& pos)
{
    return (((pos[0]-x0)*(pos[0]-x0) + (pos[1]-y0)*(pos[1]-y0)) > r*r);
}

bool isStateValid(const ob::State *state)
{
    const ob::RealVectorStateSpace::StateType& pos = *state->as<ob::RealVectorStateSpace::StateType>();
    // Valid states satisfy the following constraints:
    // -1<= x,y,z <=1
    // if .25 <= z <= .5, then |x|>.8 and |y|>.8
    // return !(fabs(pos[0])<.8 && fabs(pos[1])<.8 && pos[2]>.25 && pos[2]<.5);

    //One circle
    // return ((pos[0]*pos[0] + pos[1]*pos[1]) > 0.25);

    // multiple circles
    // float x1 = 0, y1 = 0, r1 = 0.25;
    // float x2 = 0.5, y2 = 0.5, r2 = 0.25;
    // float x3 = -0.5, y3 = 0, r3 = 0.25;
    // return (circ(x1,y1,r1,pos) && circ(x2,y2,r2,pos) && circ(x3,y3,r3,pos));
    return (circ(-0.5,0,0.45,pos));
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
    if (plannerIndex == 0){
        std::cout << "\nUsing SPARS roadmap algorithm:" << std::endl;
    	auto planner(std::make_shared<og::SPARS>(ss.getSpaceInformation()));
        ss.setPlanner(planner);
    }
    else if (plannerIndex == 1){
        std::cout << "\nUsing PRM algorithm:" << std::endl;
    	auto planner(std::make_shared<og::PRM>(ss.getSpaceInformation()));
        ss.setPlanner(planner);
    }
    else if (plannerIndex == 2){
        std::cout << "\nUsing PRM* algorithm:" << std::endl;
    	auto planner(std::make_shared<og::PRMstar>(ss.getSpaceInformation()));
        ss.setPlanner(planner);
    }
    else if (plannerIndex == 3){
        std::cout << "\nUsing RRT algorithm:" << std::endl;
        auto planner(std::make_shared<og::RRT>(ss.getSpaceInformation()));
        ss.setPlanner(planner);
    }
    else if (plannerIndex == 4){
        std::cout << "\nUsing RRT* algorithm:" << std::endl;
        auto planner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
        ss.setPlanner(planner);
    }
    else if (plannerIndex == 5){
        std::cout << "\nUsing RRT-connect algorithm:" << std::endl;
        auto planner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));
        ss.setPlanner(planner);
    }

    // attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = ss.solve(10.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        std::ofstream out;
        out.open("/home/jay/TU_Berlin_Thesis/Example_out.txt", std::ios_base::app);
        ss.getSolutionPath().printAsMatrix(out);
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::ofstream out("/home/jay/TU_Berlin_Thesis/Example_out.txt"); // create an empty txt file
    // plan(0); plan(2); plan(4);
    
    for (int i=0; i<=6; i++)
    {
        plan(i);
    }

    return 0;
}