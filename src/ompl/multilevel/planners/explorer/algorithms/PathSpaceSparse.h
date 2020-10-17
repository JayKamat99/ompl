#ifndef OMPL_MULTILEVEL_PLANNERS_BundleSpace_PathSpaceSparse_
#define OMPL_MULTILEVEL_PLANNERS_BundleSpace_PathSpaceSparse_
#include <ompl/multilevel/planners/sparse/SMLRImpl.h>
#include <ompl/multilevel/planners/explorer/datastructures/PathSpace.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(PathVisibilityChecker);

        class PathSpaceSparse : 
          public ompl::multilevel::PathSpace, 
          public ompl::multilevel::SMLRImpl
        {
            using BaseT = SMLRImpl;

        public:
            PathSpaceSparse(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);
            virtual ~PathSpaceSparse() override;
            virtual void grow() override;

        protected:
            double bestCost_{std::numeric_limits<double>::infinity()};

            PathVisibilityChecker *pathVisibilityChecker_{nullptr};
        };
    }  // namespace multilevel
}  // namespace ompl

#endif
