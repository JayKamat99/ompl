#include <ompl/geometric/PathOptimizer.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>

namespace ob = ompl::base;

ompl::geometric::PathOptimizer::PathOptimizer(base::SpaceInformationPtr si)
  : si_(std::move(si))
{}