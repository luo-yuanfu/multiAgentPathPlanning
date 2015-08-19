#ifndef VALIDITYCHECKER_H_
#define VALIDITYCHECKER_H_

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/MotionValidator.h>
#include <cmath>

namespace ob = ompl::base;
namespace og = ompl::geometric;
extern int check_num;

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si,int agent_num,double threshold) :
        ob::StateValidityChecker(si) {this->agent_num=agent_num;this->threshold=threshold;}

    bool isValid(const ob::State* state) const
    {
    	check_num++;
        if(isRobotCollide(state)) return false;

        return true;
    }

    bool isRobotCollide(const ob::State* state) const
    {
        const ompl::base::RealVectorStateSpace::StateType *state_2d= state->as<ompl::base::RealVectorStateSpace::StateType>();


        for(int i=0;i<agent_num*2-2;i=i+2)
        	for(int j=i+2;j<agent_num*2;j=j+2)
        	{
        		double x1=state_2d->values[i],y1=state_2d->values[i+1],x2=state_2d->values[j],y2=state_2d->values[j+1];
        		if(twoRobotCollide(x1,y1,x2,y2)) {return true;}
        	}


        /*
        double &x1(state_2d->values[0]),&y1(state_2d->values[1]),&x2(state_2d->values[2]),&y2(state_2d->values[3]);
        if(twoRobotCollide(x1,y1,x2,y2)) return true;
		*/
        return false;
    }

    bool twoRobotCollide(double x1, double y1, double x2, double y2) const
    {
    	//std::cout<<distanceBetwTwoRob(x1,y1,x2,y2)<<" ";
    //	if(x1>60 || y1> 40 || x2>60 || y2>40 || x1<0 || y1<0 || x2<0 || y2<0) return true;
    //	check_num++;
    //	std::cout<<x1<<" ]]]";
    //	std::cout<<this->threshold;
    	if((distanceBetwTwoRob(x1,y1,x2,y2)>3.2)) {return false;}

    	return true;
    }

    double distanceBetwTwoRob(double x1, double y1, double x2, double y2) const
    {
    	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    }


    int agent_num;
    double threshold;
};


#endif /* VALIDITYCHECKER_H_ */
