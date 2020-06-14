#include <ompl/control/optimizers/PathControlOptimizer.h>


ompl::control::PathControlOptimizer::PathControlOptimizer(base::SpaceInformationPtr si, const base::OptimizationObjectivePtr& obj)
  : si_(si), obj_(obj), freeStates_(true)
{
}
void ompl::control::PathControlOptimizer::simplify(PathControl* path)
{
  OMPL_DEBUG("simplify");
	
	//reduceVertices(*path,20,20,0.33);
	//addIntermediaryStates(*path);	
	std::vector<base::State *> &states = path->getStates();
	std::vector<Control *> &controls = path->getControls() ;
	std::vector<double > &controlDurations = path->getControlDurations() ;
	
	std::vector<base::State *> newStates ;
	std::vector<Control *> newControls ;
	std::vector<double > newControlDurations  ;
	
	const base::SpaceInformationPtr &si = path->getSpaceInformation();
	ompl::control::SpaceInformation *siC = static_cast<ompl::control::SpaceInformation*>(si.get());
	siC->setMotionValidator(std::make_shared<ompl::base::DynamicalMotionValidator>(siC));
	siC->setMinMaxControlDuration(1,50);
	
	ompl::control::ModifiedDirectedControlSamplerPtr sampler;
	sampler = siC->allocModifiedDirectedControlSampler();
	sampler->setNumControlSamples(100);
	
	base::State *state2_tmp = siC->allocState();
	std::vector<Control*>intermediaryControls ;
	std::vector<double> steps = sampler->getBestControls(intermediaryControls,states.at(0), siC->cloneState(states.at(states.size()-1)), nullptr) ;
	std::cout<< intermediaryControls.size() <<std::endl ;
	
	newStates.push_back(siC->cloneState(states.at(0)));

    for (unsigned int i =0 ; i<intermediaryControls.size(); ++ i)	
    {
		siC->propagate(states.at(i), intermediaryControls.at(i), steps.at(i) , state2_tmp) ;
		newStates.push_back(siC->cloneState(state2_tmp));	
	}
	
	
	
	states.swap(newStates) ;
	controls.swap(intermediaryControls) ;
	
	controlDurations.swap(steps) ;	
	
	
	//connectStateToGoal(0, *path, states.at(0), siC, sampler ) ; 

	/*siC->propagate(states.at(0), controls.at(0), controlDurations.at(0) , state2_tmp) ;
	siC->copyState(states.at(1), state2_tmp) ;

	siC->propagate(states.at(1), controls.at(1), controlDurations.at(1) , state2_tmp) ;
	siC->copyState(states.at(2), state2_tmp) ;	
	
	siC->propagate(states.at(2), controls.at(2), controlDurations.at(2) , state2_tmp) ;
	siC->copyState(states.at(3), state2_tmp) ;		

	
	addIntermediaryStates(*path);*/
	
	
	//controlDurations.at(0) = cD ;
	
    //ompl::control::Control *newControl =siC->allocControl() ;	
	//
	
	//double steps = sampler->sampleTo(newControl, states.at(0), states.at(1));
	//double cD = siC->propagateWhileValid(states.at(0), newControl, steps, state2_tmp);
	
	//siC->copyState(states.at(1), state2_tmp) ;
	//siC->copyControl(controls.at(0),newControl) ;
	//controlDurations.at(0)= cD ;
	
	//for (unsigned int i= 0 ; i<states.size()-1 ; ++i)
	//{
		//std::cout<< i<< std::endl ;
		//double steps = sampler->sampleTo(newControl, states.at(i), states.at(i+1));
		//double cD = siC->propagateWhileValid(states.at(i), newControl, steps, state2_tmp);
		//newControls.push_back(siC->cloneControl(newControl));	
		//newControlDurations.push_back(steps);	
	//}
	
	//controls.swap(newControls) ;
	//controlDurations.swap(newControlDurations) ;
	
	
	
	//path->check() ;
	//path->print(std::cout) ;
  OMPL_DEBUG("done simplify");
	//path->subdivide() ;
}


// void ompl::control::PathControlOptimizer::collapseCloseVertices(PathControl &path, unsigned int maxSteps, unsigned int maxEmptySteps)
// {
// 	if (path.getStateCount() < 3)
// 		return ;
 
//     if (maxSteps == 0)
//         maxSteps = path.getStateCount();
 
//     if (maxEmptySteps == 0)
//         maxEmptySteps = path.getStateCount();
 
//     const base::SpaceInformationPtr &si = path.getSpaceInformation();
//     std::vector<base::State *> &states = path.getStates();
//     ompl::control::SpaceInformation *siC = static_cast<ompl::control::SpaceInformation*>(si.get());
// 	siC->setMotionValidator(std::make_shared<ompl::base::DynamicalMotionValidator>(siC));
// 	siC->setup();
// 	siC->setMinMaxControlDuration(1,50);
 
//     // compute pair-wise distances in path (construct only half the matrix)
//     std::map<std::pair<const base::State *, const base::State *>, double> distances;
//     for (unsigned int i = 0; i < states.size(); ++i)
//         for (unsigned int j = i + 2; j < states.size(); ++j)
// 			distances[std::make_pair(states[i], states[j])] = si->distance(states[i], states[j]);
 
//     unsigned int nochange = 0;
//     for (unsigned int s = 0; s < maxSteps && nochange < maxEmptySteps; ++s, ++nochange)
//     {
//         // find closest pair of points
//         double minDist = std::numeric_limits<double>::infinity();
//         int p1 = -1;
//         int p2 = -1;
//         for (unsigned int i = 0; i < states.size(); ++i)
//             for (unsigned int j = i + 2; j < states.size(); ++j)
//             {
//                 double d = distances[std::make_pair(states[i], states[j])];
//                 if (d < minDist)
//                 {
//                     minDist = d;
//                     p1 = i;
//                     p2 = j;
//                 }
//             }
 
//         if (p1 >= 0 && p2 >= 0)
//         {
//             if (si->checkMotion(states[p1], states[p2]))
//             {
//                 if (freeStates_)
//                     for (int i = p1 + 1; i < p2; ++i)
//                         si->freeState(states[i]);
//                 states.erase(states.begin() + p1 + 1, states.begin() + p2);
//                 result = true;
//                 nochange = 0;
//             }
//             else
//                 distances[std::make_pair(states[p1], states[p2])] = std::numeric_limits<double>::infinity();
//         }
//         else
//             break;
//     }
//     return;
// }


void ompl::control::PathControlOptimizer::reduceVertices(PathControl &path, unsigned int maxSteps, unsigned int maxEmptySteps, double rangeRatio)
{
	const PathControl path_old = PathControl(path) ;
	
	if (path.getStateCount() < 3)
		return;
 
	if (maxSteps == 0)
		maxSteps = path.getStateCount();
 
	if (maxEmptySteps == 0)
		maxEmptySteps = path.getStateCount();
 
	unsigned int nochange = 0;


	std::vector<base::State *> &states = path.getStates();
	std::vector<Control *> &controls = path.getControls() ;
	std::vector<double > &controlDurations = path.getControlDurations() ;
	
	const base::SpaceInformationPtr &si = path.getSpaceInformation();
	ompl::control::SpaceInformation *siC = static_cast<ompl::control::SpaceInformation*>(si.get());
	siC->setMotionValidator(std::make_shared<ompl::base::DynamicalMotionValidator>(siC));
	siC->setMinMaxControlDuration(1,50);
	
	ompl::control::SimpleDirectedControlSamplerPtr sampler;
	sampler = siC->allocSimpleDirectedControlSampler();
	sampler->setNumControlSamples(1000);
  
	std::cout << " propagation size " << siC->getPropagationStepSize() << std::endl;
	std::cout << "initial size of states  " << states.size() << std::endl;
	std::cout << "initial size of controls  " << controls.size() << std::endl;
  
	// if (siC->checkMotion(states.front(), states.back()))
	// {
	// 	if (freeStates_)
	// 		for (std::size_t i = 2; i < states.size(); ++i)
	// 			siC->freeState(states.at(i - 1));
	// 		for (std::size_t i = 1; i < controls.size()-1; ++i)
	// 		{	
	// 			siC->freeControl(controls.at(i)) ;
	// 			controlDurations.erase(controlDurations.begin()+i) ;	
	// 		}
	// 	std::vector<base::State *> newStates(2);
    // std::vector<control::Control *> newControls;
    // std::vector<double> newControlDurations;
	// 	newControls[0] = siC->getCurrentControl();
	// 	newControlDurations[0] = siC->getControlDuration() ;
	// 	newStates[0] = states.front();
	// 	newStates[1] = states.back();	
	// 	states.swap(newStates);
	// 	controls.swap(newControls) ;
	// 	controlDurations.swap(controlDurations) ;    
	// }
	// else
	// {
	
	//ompl::base::State *stateP2_tmp = siC->allocState(); 
	//if (connectStates(45,49,path,states.at(45),siC,sampler,stateP2_tmp))
	//{
		//std::cout << "successfully connected 45 and 49" << std::endl ;
	//}
	//else 
	//{
		//std::cout << "could not connect 45 and 49" << std::endl ;
	//}
	//if (!path.check())
    //{
		//OMPL_ERROR("Path is invalid.");
        //// exit(0);
    //}
	


  for (unsigned int i = 0; i < maxSteps && nochange < maxEmptySteps; ++i, ++nochange)
  {

    int count = states.size();
    if (count<=15) 
    {
      rangeRatio= 0.5 ;
    }
    int maxN = count - 1;
    int range = 1 + (int)(floor(0.5 + (double)count * rangeRatio));

    int p1 = rng_.uniformInt(0, maxN);
    int p2 = rng_.uniformInt(std::max(p1 - range, 0), std::min(maxN, p1 + range));
    if (abs(p1 - p2) < 2)
    {
      if (p1 < maxN - 1)
        p2 = p1 + 2;
      else if (p1 > 1)
        p2 = p1 - 2;
      else
        continue;
    }

    if (p1 > p2)
      std::swap(p1, p2);
  
    if( p1 >= (int)states.size() || p2 >= (int)states.size()){
      OMPL_ERROR("p1 or p2 larger than states.");
      continue;
    }
   
		
    ompl::base::State *stateP2_tmp = siC->allocState(); 
    //connectStates(p1,p2,path,states.at(p1),siC,sampler,stateP2_tmp) ;
    
    if (connectStates(p1,p2,path,states.at(p1),siC,sampler,stateP2_tmp))
    {
		ompl::base::State *state_tmp = siC->allocState();
		
		for (unsigned int j = p2 ; j < states.size()-1 ; ++j )
		{
			if (connectStates(j,states.size()-1,path,stateP2_tmp,siC,sampler,state_tmp))
			{
				continue ;
			}
			if (connectStates(j,j+1,path,stateP2_tmp,siC,sampler,state_tmp))
			{
				siC->copyState(stateP2_tmp, state_tmp) ;
				continue ;
			}
			else
			{
				std::cout<< "could not find a connection to next or goal states at state" << j <<std::endl ;
				path = PathControl(path_old) ;
				return ;
			}
		} 
		
	}

    /*ompl::control::Control *newControl =siC->allocControl() ;
    const base::State *stateP1 = states.at(p1);
    const base::State *stateP2 = states.at(p2);
    ompl::base::State *stateP2_tmp = siC->allocState();
    siC->copyState(stateP2_tmp, stateP2);
    
    //(1) sampleTo might reach state different from P2, and this state might be
    //reached using propagate while valid.
    double steps = sampler->sampleTo(newControl, stateP1, stateP2_tmp);
    double cD = siC->propagateWhileValid(stateP1, newControl, steps, stateP2_tmp);

    //Check that we reached P2
    const double d12 = siC->getStateSpace()->distance(stateP1, stateP2);
    const double targetRegion = 0.1 * d12;

    double distToTarget = siC->getStateSpace()->distance(stateP2_tmp, stateP2);

    std::cout << "Trying shortcut: " << p1 << "  <-->  " << p2 
      << " (dist=" << distToTarget << "<=" 
      << targetRegion << ")" << std::endl;

    if(distToTarget < targetRegion)
      OMPL_DEBUG("Reached P2");
     

    if((distToTarget <= targetRegion && (cD == steps))&&(connectConsecutiveStates(p2, path,stateP2_tmp,siC,sampler)))
    {
      if (freeStates_)
      {
        for (int j = p1 + 1; j < p2; j++)
        {
          siC->freeState(states.at(j));
          siC->freeControl(controls.at(j));
        }
      }
      std::cout << "States: " << states.size() << std::endl;
      states.erase(states.begin() + p1 + 1, states.begin() + p2);	
      std::cout << "States: " << states.size() << std::endl;

      controls.erase(controls.begin() + p1 + 1, controls.begin() + p2);	
      controlDurations.erase(controlDurations.begin() + p1 + 1, controlDurations.begin() + p2);	

		//siC->printState(states.at(p1));
	  siC->copyState(states.at(p1),stateP2_tmp) ;
      siC->copyControl(controls.at(p1), newControl);
      controlDurations.at(p1) = steps;

      nochange = 0; 
      
      if (!path.check())
      {
        OMPL_ERROR("Path is invalid.");
        // exit(0);
      }
      
    } */
    // siC->freeControl(newControl);

    
  }
}

bool ompl::control::PathControlOptimizer::connectStates(unsigned int initial, unsigned int goal , ompl::control::PathControl &path, ompl::base::State* initial_State, control::SpaceInformation* siC, SimpleDirectedControlSamplerPtr sampler, ompl::base::State* reached_State) 
{
	const PathControl path_old = PathControl(path) ;
	
	if (reached_State == nullptr)
		reached_State = siC->allocState() ;
			
	
	std::vector<base::State *> &states = path.getStates();
	std::vector<Control *> &controls = path.getControls() ;
	std::vector<double > &controlDurations = path.getControlDurations() ;

    ompl::control::Control *newControl =siC->allocControl() ;
	
	base::State *state1 = siC->allocState();
	base::State *state2 = siC->allocState();
	base::State *state2_tmp = siC->allocState();
	siC->copyState(state1, initial_State);
	siC->copyState(state2, states.at(goal)); // goal -1 ??
	
	double steps = sampler->sampleTo(newControl, state1, state2);
	double cD = siC->propagateWhileValid(state1, newControl, steps, state2_tmp);

	//Check that we reached the goal
	const double d12 = siC->getStateSpace()->distance(state1, state2);
	const double targetRegion = 0.1 * d12;

	double distToTarget = siC->getStateSpace()->distance(state2_tmp, state2);
	
	if ((distToTarget > targetRegion) || (cD != steps)) 
	{
		std::cout << "could not connect the states " << initial << "and " << goal << std::endl;
		path = PathControl(path_old) ;
		return false ;
	}
	else 
	{
        for (int j = initial + 1; j<goal ; j++)
        {
          siC->freeState(states.at(j));
          siC->freeControl(controls.at(j));
        }	
        states.erase(states.begin() + initial + 1, states.begin() + goal );	
        controls.erase(controls.begin() + initial + 1, controls.begin() + goal);	
        controlDurations.erase(controlDurations.begin() + initial + 1, controlDurations.begin() + goal);	
        siC->copyState(states.at(initial), initial_State);
        siC->copyState(reached_State, state2_tmp);
		siC->copyControl(controls.at(initial) , newControl) ;
		controlDurations.at(initial)= steps ;
		
	}
	
	return true ;		
	
}

bool ompl::control::PathControlOptimizer::connectConsecutiveStates(unsigned int position, ompl::control::PathControl &path, ompl::base::State* state , 
control::SpaceInformation* siC, SimpleDirectedControlSamplerPtr sampler)
{
	return connectStates(position,position+1, path,state,siC,sampler) ;
}

bool ompl::control::PathControlOptimizer::connectStateToGoal(unsigned int position, ompl::control::PathControl &path, ompl::base::State* state, 
control::SpaceInformation* siC, SimpleDirectedControlSamplerPtr sampler ) 
{
	
	return connectStates(position, path.getStates().size()-1, path,state,siC,sampler) ;	
}


bool ompl::control::PathControlOptimizer::connectStates(unsigned int initial, unsigned int goal , ompl::control::PathControl &path, ompl::base::State* state, control::SpaceInformation* siC, SimpleDirectedControlSamplerPtr sampler) 
{
	ompl::base::State* reached_State = siC->allocState() ;
	return connectStates(initial, goal, path, state, siC, sampler, reached_State) ; 
}


void ompl::control::PathControlOptimizer::addIntermediaryStates( PathControl &path) 
{
	const base::SpaceInformationPtr &si = path.getSpaceInformation();
	ompl::control::SpaceInformation *siC = static_cast<ompl::control::SpaceInformation*>(si.get());	
	
	std::vector<base::State *> &states = path.getStates();
	std::vector<Control *> &controls = path.getControls() ;
	std::vector<double > &controlDurations = path.getControlDurations() ;
	
	std::vector<base::State *> newStates ;
	std::vector<Control *> newControls ;
	std::vector<double > newControlDurations ;
	
	unsigned int count = states.size() ;
	
	std::vector<base::State *> results  ;
	unsigned int cD ;
	newStates.push_back(states.at(0));
	
	for (unsigned int i= 0 ; i<count-1 ; ++i)
	{
		cD = siC->propagateWhileValid(states.at(i), controls.at(i), controlDurations.at(i),results,true) ;
		for (auto &state: results)
		{
			newStates.push_back(siC->cloneState(state));
			newControls.push_back(siC->cloneControl(controls.at(i)));
			newControlDurations.push_back(1) ;
		}
		siC->freeStates(results) ;
		
	}
	
	states.swap(newStates);
	controls.swap(newControls) ;
	controlDurations.swap(newControlDurations) ;
}
