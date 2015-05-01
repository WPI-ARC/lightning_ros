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

#ifndef COLLISION_CHECKER_H_
#define COLLISION_CHECKER_H_

#include <ros/ros.h>
#include <math.h>
#include <vector>

#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include "lightning/collision_utils.h"

/**
 * The CollisionChecker class interfaces with the MoveIt PlanningScene in order
 * to perform collision checking in a few situations that are useful to
 * LightningROS.
 * This class is used to provide a separate Collision Checking Service as well
 * as by the Path Shortcutting Service.
 */
class CollisionChecker {
 public:
  /**
   * Constructor.
   * @param step_size When checking the set of points between two points (See
   * checkMiddleAndReturnPoints()), this is the step size to use for the
   * itnerpolation.
   */
  CollisionChecker(double step_size);

  // No work currently done in destructor.
  ~CollisionChecker();

  /**
   * Returns a copy of the most recent PlanningScene.
   *
   * acquireScene() should be called before getPlanningScene() so that the
   * PlanningScenePtr can be locked for reading before reading, and releaseScene
   * should be called at some point afterwards to unlock the scene.
   *
   * @returns The most recent PlanningScene.
   */
  const planning_scene::PlanningScene &getPlanningScene();

  // Returns list of joint names in order.
  const std::vector<std::string> &getJointNames();

  /**
   * Acquires the lock for the PlanningScene so that CollisionChecking can be
   * done.
   * @param group_name Is the name of the group to perform collision checking on
   * (eg, "right_arm").
   * @returns success (always true).
   */
  bool acquireScene(std::string group_name);

  /**
   * Releases the lock on the current planningscene; should be done after doing
   * necessary collision checking.
   */
  void releaseScene();

  /**
   * Checks the points between first and second for collisions.
   *
   * This interpolates between first and second with a step size of step_size_
   * and then returns true if all the interpolated states are actually valid. If
   * they are valid, then new_points will be populated with the interpolation.
   * This function is used by the path shortcutting to check whether it is
   * possible to interpolate between two points and to actually do the
   * interpolation.
   *
   * @param[in] first The first of the pair of points to check between.
   * @param[in] second The second of the pair of points to check between. Should
   * have the same number of elements as first.
   * @param[out] new_points The interpolation between the two points; empty of
   * collision checking failed.
   * @returns true if path is collision free; false otherwise.
   */
  bool checkMiddleAndReturnPoints(
      const std::vector<double> &first, const std::vector<double> &second,
      std::vector<std::vector<double> > &new_points);

  /**
   * Checks the validity of the provided state and provides additional
   * functionality for multi-threading.
   *
   * @param state The particular state to check; should have a length of
   * num_joints_.
   * @param clone Whether or not there is already a collision check running.
   * This should be used for multi-threading, when the main PlanningScene may be
   * being used and so we need to copy the current PlanningScene to avoid
   * interfering with existing runs. Setting clone to true will not cause any
   * issues if you are not multi-threading.
   * @returns true of the state is valid.
   */
  bool isStateValid(
      const ::std::vector<double> &state,
      bool clone =
          false /*Whether we will need to create a copy of the planningscene before proceeding*/);

 private:
  // PlanningSceneMonitor which deals with updating the PlanningScene whenever
  // the environment changes (eg, obstacles added).
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;

  // PlanningScenePtr which refers to the PlanningScene being updated by psm_.
  planning_scene::PlanningScenePtr ps_;

  // Information about current group being collision checked.
  std::vector<std::string> arm_names_;
  std::vector<std::string> joint_names_;
  std::string group_name_;
  int num_joints_;

  // When interpolating, distance between each new point.
  double step_size_;
};

#endif  // for COLLISION_CHECKER_H_
