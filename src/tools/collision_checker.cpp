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
  // Progressively set up monitor, create a pointer to it, create a safer
  // pointer using LockedScene (ls_).
  psm_.reset(new planning_scene_monitor::PlanningSceneMonitor("/robot_description"));
  ROS_INFO("Constructed psm_.");
  ps_ = psm_->getPlanningScene();
  // Right now, all we really care about is whether there is collision or not;
  // nothing fancy, so defaults for everything not explicitly constructed should
  // be fine-ish.

  step_size_ = step_size;

  // TODO: allow overriding of default service and topic subscriptions for psm_.
  // Get initial planning scene state from /get_planning_scene service.
  psm_->requestPlanningSceneState("/get_planning_scene");
  ROS_INFO("Request from psm_.");
  psm_->startSceneMonitor("/move_group/monitored_planning_scene"); // Default "/planning_scene"
}

CollisionChecker::~CollisionChecker() {}

bool CollisionChecker::collisionModelsInterfaceLoadedModels() {
  // TODO: Figure out appropriate behavior with MoveIt.
  return true;//collision_models_interface_->loadedModels();
}

const planning_scene::PlanningScene &CollisionChecker::getPlanningScene() {
  return *ps_;
}

const std::vector<std::string> &CollisionChecker::getJointNames() {
  return joint_names_;
}

bool CollisionChecker::acquireScene(std::string group_name) {
  // May need to manually call some sort of lock.
  //if (!collision_models_interface_->isPlanningSceneSet()) {
  //  ROS_WARN("Collision checker: Calling with no planning scene set");
  //  collision_models_interface_->bodiesUnlock();
  //  return false;
  //}
  psm_->lockSceneRead();
  arm_names_ = ps_->getCurrentState()
                   .getJointModelGroup(group_name)
                   ->getUpdatedLinkModelNames();
  joint_names_ = ps_->getCurrentState()
                     .getJointModelGroup(group_name)
                     ->getJointModelNames();
  num_joints_ = joint_names_.size();
  //collision_models_interface_->resetToStartState(
  //    *(collision_models_interface_->getPlanningSceneState()));
  return true;
}

void CollisionChecker::releaseScene() {
  // Figure out if this needs to do anything under MoveIt.
  psm_->unlockSceneRead();
}

// In old interface, must call acquireScene before checking a path and
// releaseScene after checking a path. May not need to in new. Do so just to be
// safe, in case I end up messing around with this stuff.

// if the middle is clear, then store the middle points in new_points
// if the middle is not clear, then new_points is cleared
bool CollisionChecker::checkMiddleAndReturnPoints(
    const std::vector<double> &first, const std::vector<double> &second,
    std::vector<std::vector<double> > &new_points) {

  new_points.clear();
  std::vector<std::vector<double> > interpolation =
      interpolate(first, second, step_size_);
  for (int i = 0; i < (int)interpolation.size(); i++) {
    if (!isStateValid(interpolation[i])) return false;
  }
  new_points = interpolation;
  return true;
}

// Call with a list of joint positions corresponding to what the current
// joint_names are.
bool CollisionChecker::isStateValid(const ::std::vector<double> &state) {
  moveit::core::RobotState robot_state = ps_->getCurrentState();
  std::map<std::string, double> pos;
  for (int i = 0; i < num_joints_; i++) {
    pos[joint_names_[i]] = state[i];
  }
  robot_state.setVariablePositions(pos);
  // Not sure if this actually checks for joint limits correctly.
  return ps_->isStateValid(robot_state);
}
