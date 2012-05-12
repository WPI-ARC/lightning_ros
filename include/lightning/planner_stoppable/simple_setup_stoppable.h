#ifndef SIMPLE_SETUP_STOPPABLE_H
#define SIMPLE_SETUP_STOPPABLE_H

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/StateSpace.h>
#include <ompl/util/Time.h>
#include <ompl/base/PlannerTerminationCondition.h>

class SimpleSetupStoppable : public ompl::geometric::SimpleSetup {
    public:
        SimpleSetupStoppable(const ompl::base::StateSpacePtr &space);
        //~SimpleSetupStoppable();
        virtual bool solve(ompl::base::PlannerTerminationCondition &ptc);
};

#endif 
