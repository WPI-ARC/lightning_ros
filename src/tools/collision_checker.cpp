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

CollisionChecker::CollisionChecker(double step_size) : step_size_(step_size) {
  // Create the PlanningSceneMonitor and then set the PlanningScenePtr to be
  // driven by the Monitor.
  psm_.reset(new planning_scene_monitor::PlanningSceneMonitor("/robot_description"));
  ps_ = psm_->getPlanningScene();

  // TODO: allow overriding of default service and topic subscriptions for psm_.

  // Get initial planning scene state from /get_planning_scene service.
  psm_->requestPlanningSceneState("/get_planning_scene");
  // Monitor /planning_scene for any future changes to the scene.
  psm_->startSceneMonitor("/planning_scene");
}

CollisionChecker::~CollisionChecker() {}

const planning_scene::PlanningScene &CollisionChecker::getPlanningScene() {
  return *ps_;
}

const std::vector<std::string> &CollisionChecker::getJointNames() {
  return joint_names_;
}

bool CollisionChecker::acquireScene(std::string group_name) {
  group_name_ = group_name;
  // Lock scene monitor for reading.
  psm_->lockSceneRead();
  // Retrieve information about the provided group.
  arm_names_ = ps_->getCurrentState()
                   .getJointModelGroup(group_name)
                   ->getUpdatedLinkModelNames();
  joint_names_ = ps_->getCurrentState()
                     .getJointModelGroup(group_name)
                     ->getActiveJointModelNames();
  num_joints_ = joint_names_.size();
  return true;
}

void CollisionChecker::releaseScene() {
  // Figure out if this needs to do anything under MoveIt.
  psm_->unlockSceneRead();
}

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

bool CollisionChecker::isStateValid(const ::std::vector<double> &state,
                                    bool clone) {
  // Perform the copy in case multi-threading is needed.
  planning_scene::PlanningScenePtr scene =
      clone ? planning_scene::PlanningScene::clone(ps_) : ps_;

  moveit::core::RobotState robot_state = scene->getCurrentState();
  std::map<std::string, double> pos;
  for (int i = 0; i < num_joints_; i++) {
    pos[joint_names_[i]] = state[i];
    ROS_INFO("Setting %s to %f", joint_names_[i].c_str(), state[i]);
  }
  robot_state.setVariablePositions(pos);
  scene->setCurrentState(robot_state);

  // This checks for self-collision and environment collision. The
  // isStateValid() method of PlanningScene performs extra checks which we do
  // not particularly need.
  return !scene->isStateColliding(robot_state, group_name_, false/*verbose*/);
}
