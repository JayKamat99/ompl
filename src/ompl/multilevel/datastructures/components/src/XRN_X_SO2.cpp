#include <ompl/multilevel/datastructures/components/XRN_X_SO2.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

ompl::multilevel::BundleSpaceComponent_SO2RN_SO2::BundleSpaceComponent_SO2RN_SO2(base::StateSpacePtr BundleSpace,
                                                                                base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}

void ompl::multilevel::BundleSpaceComponent_SO2RN_SO2::projectBase(const ompl::base::State *xBundle,
                                                                  ompl::base::State *xBase) const
{
    const base::SO2StateSpace::StateType *xBundle_SO2 =
        xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    base::SO2StateSpace::StateType *xBase_SO2 = xBase->as<base::SO2StateSpace::StateType>();

    xBase_SO2->value = xBundle_SO2->value;
}

void ompl::multilevel::BundleSpaceComponent_SO2RN_SO2::liftState(const ompl::base::State *xBase,
                                                                const ompl::base::State *xFiber,
                                                                ompl::base::State *xBundle) const
{
    base::SO2StateSpace::StateType *xBundle_SO2 =
        xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::SO2StateSpace::StateType *xBase_SO2 = xBase->as<base::SO2StateSpace::StateType>();
    const base::RealVectorStateSpace::StateType *xFiber_RN = xFiber->as<base::RealVectorStateSpace::StateType>();

    xBundle_SO2->value = xBase_SO2->value;

    for (unsigned int k = 0; k < getFiberDimension(); k++)
    {
        xBundle_RN->values[k] = xFiber_RN->values[k];
    }
}
