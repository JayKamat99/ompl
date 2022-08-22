#include <ompl/base/ProblemDefinition_ext.h>

namespace ompl
{
    namespace base
    {
        void ProblemDefinition_ext::addReachedGoal(const ompl::base::State* goal_, double costToReach, ompl::geometric::PathGeometricPtr pathGeoPtr)
        {
            std::cout << "addReachedGoal called!" << std::endl;
            std::cout << goal_ << std::endl;
            bool goalAccounted = false;
            for(auto goal:reachedGoals)
            {
                if (goal == goal_)
                    goalAccounted = true;
            }
            if (!goalAccounted)
                reachedGoals.push_back(goal_);
                
            goalCost[goal_] = costToReach;
            path[goal_] = pathGeoPtr;
        }

        ompl::geometric::PathGeometricPtr ProblemDefinition_ext::getPath(const ompl::base::State* goal_)
        {
            return path.at(goal_);
        }

        double ProblemDefinition_ext::getCostToReach(const ompl::base::State* goal_)
        {
            for (auto i:reachedGoals)
            {
                if (i == goal_)
                    return goalCost.at(goal_);
            }
            return INFINITY;
        }

        // save it as state*
    } // namespace base
} //namespace ompl