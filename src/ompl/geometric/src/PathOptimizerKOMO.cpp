#include <ompl/geometric/PathOptimizer.h>
#include <ompl/geometric/PathOptimizerKOMO.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

ompl::geometric::PathOptimizerKOMO::PathOptimizerKOMO(base::SpaceInformationPtr si): si_(std::move(si)) {}

bool ompl::geometric::PathOptimizerKOMO::optimize(PathGeometric &path)
{
	std::cout << "Yay! you did it!" << std::endl;
	return true;

	arrA configs;
	//To copy the path to arrA Configs from states.
	const base::StateSpace *space(si_->getStateSpace().get());
	for (auto state : path.getStates())
		{
			arr config;
			std::vector<double> reals;
			space->copyToReals(reals, state);
			for (double r : reals){
				config.append(r);
			}
			configs.append(config);
	}

    std::cout << configs.N << "before" << std::endl;

    // Create a text string, which is used to output the text file
    std::string filename;
    ifstream MyReadFile("/home/jay/git/optimization-course/examples/Models/Configuration.txt");
    getline (MyReadFile, filename);
    MyReadFile.close(); 

    // setup KOMO
    rai::Configuration C;
    C.addFile(filename.c_str());
    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    
    komo.setTiming(1., configs.N, 5., 2);
    komo.add_qControlObjective({}, 1, 50.);

    // std::cout << configs << std::endl;
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
	for (auto state : path.getStates())
    {
		std::vector<double> reals;
		for (double r : configs(i)){
			reals.push_back(r);
		}
		space->copyFromReals(state, reals);
		i++;
    }

    return true; //this is useless
}