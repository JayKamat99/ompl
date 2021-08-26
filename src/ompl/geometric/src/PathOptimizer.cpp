// #include <ompl/geometric/PathOptimizer.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>

namespace ob = ompl::base;

bool ompl::geometric::PathSimplifier::optimizePathKOMO(PathGeometric &path)
{
    arrA configs;
    //To copy the path to arrA Config from states.
    for (auto &s : path.getStates()) {
      const auto *State = s->as<ob::SE2StateSpace::StateType>();

      arr x_query = arr
      {
        State->getX(),
        State->getY(),
        State->getYaw()
      };
      configs.append(x_query);
    }

    std::cout << configs.N << "before" << std::endl;

    auto filename = "/home/jay/git/optimization-course/examples/Models/2D_bot.g";
    rai::Configuration C;
    C.addFile(filename);
    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    
    komo.setTiming(1., configs.N, 5., 2);
    komo.add_qControlObjective({}, 1, 50.);

    std::cout << configs << std::endl;
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1}, configs(configs.N-1), 0);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});
    komo.add_collision(true);

    //use configs to initialize with waypoints
    komo.initWithWaypoints(configs, path.getStateCount(), false);
    komo.run_prepare(0);
    // komo.view(true);
    // komo.view_play(true);
    komo.optimize();
    // komo.view(true);
    // komo.view_play(true);

    configs = komo.getPath_q();
    
    std::cout << configs.N << "after" << std::endl;
    
    //copy the final config back to states
    int i=0;
    for (auto &s : path.getStates()) {
        auto *State = s->as<ob::SE2StateSpace::StateType>();
        State->setX((configs(i))(0));
        State->setY((configs(i))(1));
        State->setYaw((configs(i))(2));
        i++;
    }

    return true; //this is useless
}