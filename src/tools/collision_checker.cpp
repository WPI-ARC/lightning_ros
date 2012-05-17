/*
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, University of California, Berkeley
# All rights reserved.
# Authors: Cameron Lee (cameronlee@berkeley.edu) and Dmitry Berenson (
berenson@eecs.berkeley.edu)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of California, Berkeley nor the names 
of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
*/

#include "lightning/collision_checker.h"

CollisionChecker::CollisionChecker(double step_size) {
    collision_models_interface_ = new planning_environment::CollisionModelsInterface("/robot_description");

    step_size_ = step_size;
}

CollisionChecker::~CollisionChecker() {   
    delete collision_models_interface_;
}

bool CollisionChecker::collisionModelsInterfaceLoadedModels() {
    return collision_models_interface_->loadedModels();
}

const arm_navigation_msgs::PlanningScene& CollisionChecker::getPlanningScene() {
    return collision_models_interface_->getLastPlanningScene();
}

const std::vector<std::string>& CollisionChecker::getJointNames() {
    return joint_names_;
}

bool CollisionChecker::acquireScene(std::string group_name) {
    collision_models_interface_->bodiesLock();
    if(!collision_models_interface_->isPlanningSceneSet()) {
        ROS_WARN("Collision checker: Calling with no planning scene set");
        collision_models_interface_->bodiesUnlock();
        return false;
    }
    arm_names_ = collision_models_interface_->getKinematicModel()->getModelGroup(group_name)->getUpdatedLinkModelNames();
    joint_names_ = collision_models_interface_->getKinematicModel()->getModelGroup(group_name)->getJointModelNames();
    num_joints_ = joint_names_.size();
    collision_models_interface_->resetToStartState(*(collision_models_interface_->getPlanningSceneState()));
    return true;
}

void CollisionChecker::releaseScene() {
    collision_models_interface_->bodiesUnlock();
}

//must call acquireScene before checking a path and releaseScene after checking a path
bool CollisionChecker::checkStateValid(const std::vector<double> &point) {
    planning_models::KinematicState state = *collision_models_interface_->getPlanningSceneState();
    std::map<std::string, double> pos;
    for (int i = 0; i < num_joints_; i++) {
        pos[joint_names_[i]] = point[i];
    }
    state.setKinematicState(pos);

    return isStateValid(state);
}

bool CollisionChecker::checkMiddle(const std::vector<double> &first, const std::vector<double> &second) {
    planning_models::KinematicState state = *collision_models_interface_->getPlanningSceneState();
    std::map<std::string, double> pos;
    std::vector< std::vector<double> > interpolation = interpolate(first, second, step_size_);
    for (int i = 0; i < (int)interpolation.size(); i++) {
        for (int j = 0; j < num_joints_; j++) {
            pos[joint_names_[j]] = interpolation[i][j];
        }
        state.setKinematicState(pos);
        if (!isStateValid(state)) {
            return false;
        }
    }
    return true;
}

//if the middle is clear, then store the middle points in new_points
//if the middle is not clear, then new_points is cleared
bool CollisionChecker::checkMiddleAndReturnPoints(const std::vector<double> &first, const std::vector<double> &second, std::vector< std::vector<double> > &new_points) {
    planning_models::KinematicState state = *collision_models_interface_->getPlanningSceneState();
    new_points.clear();
    std::map<std::string, double> pos;
    std::vector< std::vector<double> > interpolation = interpolate(first, second, step_size_);
    for (int i = 0; i < (int)interpolation.size(); i++) {
        for (int j = 0; j < num_joints_; j++) {
            pos[joint_names_[j]] = interpolation[i][j];
        }
        state.setKinematicState(pos);
        if (!isStateValid(state)) {
            return false;
        }
    }
    new_points = interpolation;
    return true;
}

bool CollisionChecker::isStateValid(const planning_models::KinematicState &state) {
    if (!state.areJointsWithinBounds(joint_names_)) {
        return false;
    } else if (collision_models_interface_->isKinematicStateInCollision(state)) {
        return false;
    } else {
        return true;
    }
}
