#ifndef OMPL_GEOMETRIC_PATH_OPTIMIZER_
#define OMPL_GEOMETRIC_PATH_OPTIMIZER_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathOptimizer);

        class PathOptimizer
        {
        public:
            PathOptimizer(base::SpaceInformationPtr si);
            virtual ~PathOptimizer() = default;

            virtual bool optimize(PathGeometric &path)=0;

        protected:
            /** \brief The space information this path simplifier uses */
            base::SpaceInformationPtr si_;
        };
    } // namespace geometric
} // namespace ompl


#endif