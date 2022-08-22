#ifndef OMPL_BASE_PROBLEM_DEFINITION_EXT_
#define OMPL_BASE_PROBLEM_DEFINITION_EXT_

#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/PathGeometric.h>
#include <set>
#include <map>
#include <vector>

/**
 * @brief The reason for having this extension is add more functions like,
 * 1) Asking for a solution path given goal state.
 * 
 */

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(ProblemDefinition_ext);

        class ProblemDefinition_ext : public base::ProblemDefinition
        {
        private:
            std::map<const ompl::base::State*, double> goalCost;
            std::map<const ompl::base::State*, ompl::geometric::PathGeometricPtr> path;
            std::vector<const ompl::base::State*> reachedGoals;
        public:
            ProblemDefinition_ext(SpaceInformationPtr si) : ProblemDefinition(si) {}
            virtual ~ProblemDefinition_ext() override {std::cout << "pdef destroyed" << std::endl;}

            /* Modifies the costToReach and Path to the goal. Adds the goal state to the vector of goal state only if it is a new goal */
            void addReachedGoal(const ompl::base::State* goal_, double costToReach, ompl::geometric::PathGeometricPtr pathGeoPtr);
            ompl::geometric::PathGeometricPtr getPath(const ompl::base::State* goal_);
            double getCostToReach(const ompl::base::State* goal_);
            std::vector<const ompl::base::State*> get_reachedGoals() {return reachedGoals;}
        };  
    } // namespace base
} //namespace ompl

#endif