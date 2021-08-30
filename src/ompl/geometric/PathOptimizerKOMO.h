#ifndef OMPL_GEOMETRIC_PATH_OPTIMIZER_KOMO_
#define OMPL_GEOMETRIC_PATH_OPTIMIZER_KOMO_

#include <PathOptimizer.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>

namespace ompl
{
    namespace geometric
    {
        class PathOptimizerKOMO : public PathOptimizer
        {
        private:
            /* data */
        public:
            using PathOptimizer::PathOptimizer;
            ~PathOptimizerKOMO();

            bool optimize(PathGeometric &path) override;
        };  
    } // namespace  geometric
} //namespace ompl

#endif