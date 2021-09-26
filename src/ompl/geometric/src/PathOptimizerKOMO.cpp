#include <ompl/geometric/PathOptimizer.h>
#include <ompl/geometric/PathOptimizerKOMO.h>
#include <ompl/multilevel/planners/multimodal/PathSpaceSparse.h>
#include <ompl/multilevel/planners/multimodal/datastructures/PathSpace.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>
#include <Core/graph.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace om = ompl::multilevel;

ompl::geometric::PathOptimizerKOMO::PathOptimizerKOMO(base::SpaceInformationPtr si): si_(std::move(si)) {}

bool ompl::geometric::PathOptimizerKOMO::optimize(PathGeometric &path)
{
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
    komo.add_qControlObjective({}, 1, 2.);

    // std::cout << configs << std::endl;
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10}, configs(configs.N-1), 0);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});
    komo.add_collision(true);

    //use configs to initialize with waypoints
    komo.initWithWaypoints(configs, path.getStateCount(), false);
    komo.run_prepare(0);
    komo.animateOptimization = 1;
    komo.optimize();
	rai::Graph R = komo.getReport(false);
 	double constraint_violation = R.get<double>("eq") + R.get<double>("ineq");
    std::cout << "Constraint Violations:" << constraint_violation << std::endl;

    configs = komo.getPath_q();
    
    std::cout << configs.N << "after" << std::endl;

    bool isValid = true;
    if (constraint_violation > 1){
        isValid = false;
        for (int i=0; i<=7; i++){
            std::string frame = "iiwa_link_"+std::to_string(i)+"_1";
            komo.pathConfig.getFrame(frame.c_str())->setColor({1,0,0});
        }
        for (int j=0; j<10 ;j++)
            komo.displayPath("Infeasible", false);
    }

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
	isStepWise = false;
    setPathCost(R.get<double>("sos"));
    return isValid;
}

void ompl::geometric::PathOptimizerKOMO::displayPath(PathGeometric &path, const std::string &txt) const
{
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
    komo.add_qControlObjective({}, 1, 2.);

    komo.initWithWaypoints(configs, path.getStateCount(), false);

    for (int i=0; i<=7; i++){
        std::string frame = "iiwa_link_"+std::to_string(i)+"_1";
        if (txt == "Repeated")
            komo.pathConfig.getFrame(frame.c_str())->setColor({1,1,0});
        else
            komo.pathConfig.getFrame(frame.c_str())->setColor({0,1,0});
    }
    for (int j=0; j<10 ;j++)
        komo.displayPath(txt.c_str(), false);
}