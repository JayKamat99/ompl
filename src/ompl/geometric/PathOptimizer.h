#ifndef OMPL_GEOMETRIC_PATH_OPTIMIZER_
#define OMPL_GEOMETRIC_PATH_OPTIMIZER_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/OptimizationObjective.h>

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathOptimizer);

        class PathOptimizer
        {
        private:
            /* data */
        public:
            PathOptimizer(base::SpaceInformationPtr si, const base::GoalPtr &goal = ompl::base::GoalPtr(), const base::OptimizationObjectivePtr& obj=nullptr);
            virtual ~PathOptimizer() = default;

            /** \brief Given a path this method attempts to find a better path to the goal.
             **/
            bool optimizePathKOMO();
        };
        
    } // namespace geometric
    
} // namespace ompl


#endif