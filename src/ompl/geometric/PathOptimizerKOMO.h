#ifndef OMPL_GEOMETRIC_PATH_OPTIMIZER_KOMO_
#define OMPL_GEOMETRIC_PATH_OPTIMIZER_KOMO_

#include <ompl/geometric/PathOptimizer.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathOptimizerKOMO);

        class PathOptimizerKOMO : public PathOptimizer
        {
        private:
            /* data */

        public:
            // using PathOptimizer::PathOptimizer;
            PathOptimizerKOMO(base::SpaceInformationPtr si);
            virtual ~PathOptimizerKOMO() = default;

            bool optimize(PathGeometric &path) override;
            void optimize() override {
                std::cout<<"You are on the right track" << std::endl;
            }

        protected:
            base::SpaceInformationPtr si_;
        };  
    } // namespace  geometric
} //namespace ompl

#endif