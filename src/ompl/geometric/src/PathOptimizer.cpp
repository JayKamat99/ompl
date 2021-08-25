// #include <ompl/geometric/PathOptimizer.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>

namespace ob = ompl::base;

static arrA configs;

bool ompl::geometric::PathSimplifier::optimizePathKOMO(PathGeometric &path)
{
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
    komo.setModel(C, true);
    
    komo.setTiming(1., configs.N, 5., 2);
    komo.add_qControlObjective({}, 1, 50.);

    komo.addObjective({0.98, 1.}, FS_qItself, {},OT_eq, {1}, {1,1,0}, 0);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});
    komo.add_collision(true);

    //use configs to initialize with waypoints
    komo.initWithWaypoints(configs, path.getStateCount(), false);
    komo.optimize();
    komo.plotTrajectory();
    komo.checkGradients();
    rai::ConfigurationViewer V;
    V.setPath(C, komo.x, "result", false);
    while(V.playVideo());
    
    configs = komo.getPath_q();
    
    std::cout << configs.N << "after" << std::endl;
    
    //copy the final config back to states
    for (auto &s : path.getStates()) {
        auto *State = s->as<ob::SE2StateSpace::StateType>();
        State->setX(0);
        State->setY(0);
        State->setYaw(0);
    }
}