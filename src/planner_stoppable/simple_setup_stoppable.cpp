#include "lightning/planner_stoppable/simple_setup_stoppable.h"

SimpleSetupStoppable::SimpleSetupStoppable(const ompl::base::StateSpacePtr &space)
    : SimpleSetup(space)
{
    //psk_.reset(new PathSimplifierShortcutter(si_));
}

//most of this function is from SimpleSetup.cpp in ompl version 0.10 (by Ioan Sucan)
bool SimpleSetupStoppable::solve(ompl::base::PlannerTerminationCondition &ptc) {
    setup();
    ompl::time::point start = ompl::time::now();
    bool result = planner_->solve(ptc);
    planTime_ = ompl::time::seconds(ompl::time::now() - start);
    if (result)
        msg_.inform("Solution found in %f seconds", planTime_);
    else
        msg_.inform("No solution found after %f seconds", planTime_);
    return result;
}

