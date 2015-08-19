#include "MARRTstar.h"
#include "validityChecker.h"
#include "myMotionValidator.h"
#include "draw.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
//#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/MotionValidator.h>
#include <cmath>
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

int check_num=0;
void planWithSimpleSetup(int agent_num)
{
  // Construct the state space where we are planning

  ob::StateSpacePtr space(new ob::RealVectorStateSpace(2*agent_num));

  ob::RealVectorBounds bounds(2*agent_num);
  for(int j=0;j<agent_num;j++)
  {
	   //x dimension
	   bounds.setLow(2*j,0);
	   bounds.setHigh(2*j,60);

	   //y dimension
	   bounds.setLow(2*j+1,0);
	   bounds.setHigh(2*j+1,40);

  }


  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  // Instantiate SimpleSetup
  og::SimpleSetup ss(space);

  // Setup the StateValidityChecker
//  ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
  ompl::base::SpaceInformationPtr si=ss.getSpaceInformation();

  si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si,agent_num,(bounds.high[0]-bounds.low[0])*0.06)));
  si->setup();
 // ss.setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));

  si->setMotionValidator(ob::MotionValidatorPtr(new myMotionValidator(si,agent_num)));
 // si->setup();


  // Setup Start and Goal
  ob::ScopedState<ob::RealVectorStateSpace> start(space);


  start->values[0]=5.0;
  start->values[1]=5.0;
  if(agent_num>=2)
  {
	  start->values[2]=40;
	  start->values[3]=10;

  }
  if(agent_num>=3)
  {
   	 start->values[4]=32.0;
   	 start->values[5]=8.0;
  }
  if(agent_num>=4)
   {
    	 start->values[6]=50.0;
    	 start->values[7]=8.0;
   }
  if(agent_num>=5)
   {
    	 start->values[8]=46.0;
    	 start->values[9]=32.0;
   }



//  start.random();
  std::cout << "start: "; start.print(std::cout);

  ob::ScopedState<ob::RealVectorStateSpace> goal(space);
 // goal->setXY(0.9,0.9);

  goal->values[0]=46.0;
  goal->values[1]=32.0;
  if(agent_num>=2)
  {
	  goal->values[2]=10;
	  goal->values[3]=23.0;
  }

  if(agent_num>=3)
  {
	  goal->values[4]=33.0;
	  goal->values[5]=30.0;
  }

  if(agent_num>=4)
   {
 	  goal->values[6]=33.0;
 	  goal->values[7]=8.0;
   }

  if(agent_num>=5)
   {
 	  goal->values[8]=46.0;
 	  goal->values[9]=10.0;
   }


//  goal.random();
  std::cout << "goal: "; goal.print(std::cout);

  ss.setStartAndGoalStates(start, goal);

  std::cout << "----------------" << std::endl;

  //ompl::base::PlannerPtr planner(new ompl::geometric::RRT(si));
//  ompl::geometric::RRT *my_rrt=new ompl::geometric::RRT(si);
  ompl::geometric::MaRRTstar *my_rrt=new ompl::geometric::MaRRTstar(si);
  my_rrt->setRange(0.4);
  my_rrt->setup();
  ompl::base::PlannerPtr planner=ompl::base::PlannerPtr(my_rrt);

  ss.setPlanner(planner);
  si->setStateValidityCheckingResolution(0.1);
 // si->setMotionValidator(ob::MotionValidatorPtr(new myMotionValidator(si)));
  ss.setup();

  // Execute the planning algorithm
  ob::PlannerStatus solved = ss.solve(5);

  // If we have a solution,
  if (solved)
  {
    // Simplify the solution
	//ss.simplifySolution();
    std::cout << "----------------" << std::endl;
    std::cout << "Found solution:" << std::endl;
    // Print the solution path to screen

    ss.getSolutionPath().print(std::cout);

    // Print the solution path to a file
    std::ofstream ofs("path.dat");
    ss.getSolutionPath().printAsMatrix(ofs);
    ofs.close();
  }
  else
    std::cout << "No solution found" << std::endl;


  /*
   * if want to plot the path using python to plot it, copy the following code
import numpy;
import matplotlib.pyplot as plt;
data=numpy.loadtxt('path.dat');
plt.plot(data[:,0],data[:,1],'.-');
plt.show();
plt.plot(data[:,0],data[:,1]);
plt.show();
   *
   */
}


int main()
{
	int agent_num=1;
out2:	std::cout<<"please input the number of agents(1-5):"<<std::endl;
	std::cin>>agent_num;
	if(agent_num>5 || agent_num<1) {std::cout<<"please input the number ranging (1,5):"<<std::endl; goto out2;}

	planWithSimpleSetup(agent_num);
	std::cout<<"check num: "<<check_num;
	drawMap(60.0,40.0,agent_num);
	return 0;
}



