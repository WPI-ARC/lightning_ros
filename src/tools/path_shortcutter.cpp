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

#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <cstdlib>
#include "lightning/PathShortcut.h"
#include "lightning/Float64Array.h"
#include "lightning/Status.h"

#include "lightning/collision_checker.h"
#include "lightning/collision_utils.h"

/**
 * The PathShortcutter advertises a service which will go through a path and
 * randomly select pairs of points and attempt to interpolate straight between
 * them to simplify the path.
 */
class PathShortcutter {
 public:
  PathShortcutter() : private_handle_("~") {
    // Retrieve settings from parameter server.
    // The step size is the distance between interpolated points when
    // interpolating and collision checking.
    handle_.getParam("step_size", step_size_);
    // num_iters_ is the number of random pairs of points to attempt to shortcut
    // between.
    private_handle_.getParam("num_iterations", num_iters_);
    // When path shortcutting, if we don't have the potential of saving at least
    // as much as this fraction of the distance, then don't bother even doing
    // the collision checking.
    private_handle_.getParam("ignore_fraction", distance_fraction_);

    // Wait until the collision checker can actually retrieve a valid planning
    // scene.
    ros::service::waitForService("/get_planning_scene");
    collision_checker_ = new CollisionChecker(step_size_);
  }

  ~PathShortcutter() { delete collision_checker_; }

  // Actually advertise the service.
  void run() {
    service_ = handle_.advertiseService("shortcut_path",
                                        &PathShortcutter::shortcutPath, this);
  }

  // Callback for the actual shortcutting service.
  bool shortcutPath(lightning::PathShortcut::Request &req,
                    lightning::PathShortcut::Response &res) {
    res.status.status = lightning::Status::FAILURE;
    ROS_INFO("Path shortcutter service: got a path with %u points",
             (int)req.path.size());

    std::vector<std::vector<double> > orig_path;
    for (int i = 0; i < (int)req.path.size(); i++) {
      orig_path.push_back(req.path[i].values);
    }

    // Perform actual shortcutting.
    if (collision_checker_->acquireScene(req.group_name)) {
      std::vector<std::vector<double> > short_path = doShortcutting(orig_path);
      collision_checker_->releaseScene();

      short_path = rediscretizePath(short_path, step_size_);

      ROS_INFO("Path shortcutter service: new path has %u points",
               (int)short_path.size());
      lightning::Float64Array float_array;
      for (int i = 0; i < (int)short_path.size(); i++) {
        float_array.values = short_path[i];
        res.new_path.push_back(float_array);
      }

      res.status.status = lightning::Status::SUCCESS;
    }

    return true;
  }

 private:
  ros::NodeHandle private_handle_, handle_;
  CollisionChecker *collision_checker_;
  ros::ServiceServer service_;

  // Configuration variables loaded from parameter server; see constructor.
  double step_size_;
  int num_iters_;
  double distance_fraction_;

  // returns a random integer in the interval [0, max_value)
  int getRandWithMax(int max_value) {
    return (int)(max_value * (rand() / (RAND_MAX + 1.0)));
  }

  // Get distance along path between indicies index1 and index 2.
  double getFullDistance(std::vector<std::vector<double> > &path, int index1,
                         int index2) {
    double dist = 0;
    for (int i = index1; i < index2; i++) {
      dist += getLineDistance(path[i], path[i + 1]);
    }
    return dist;
  }

  // do shortcutting and return shortcutted path.
  std::vector<std::vector<double> > doShortcutting(
      std::vector<std::vector<double> > &orig_path) {
    int rand1, rand2, temp;
    std::vector<std::vector<double> > new_path = orig_path;
    std::vector<std::vector<double> > new_points;

    // Iterate through; each time, chose a random pair of points and try
    // shortcutting.
    for (int i = 0; i < num_iters_; i++) {
      rand1 = getRandWithMax(new_path.size());
      rand2 = getRandWithMax(new_path.size());
      if (rand1 > rand2) {
        temp = rand1;
        rand1 = rand2;
        rand2 = temp;
      }
      // Don't even bother collision checking unless rand2 and rand1 are both
      // not already next to each other and so long as there is an appreciable
      // amount of distance that we actually of the potential to save.
      if (rand2 - rand1 > 1 &&
          (getFullDistance(new_path, rand1, rand2) >
           (1.0 + distance_fraction_) *
               getLineDistance(new_path[rand1], new_path[rand2]))) {
        // Actually do collision check and update path if interpolation is valid
        if (collision_checker_->checkMiddleAndReturnPoints(
                new_path[rand1], new_path[rand2], new_points)) {
          new_path.erase(new_path.begin() + rand1 + 1,
                         new_path.begin() + rand2);
          new_path.insert(new_path.begin() + rand1 + 1, new_points.begin(),
                          new_points.end());
          ROS_INFO(
              "path shortcutter: iteration %u shortcut a section of %u points",
              i, rand2 - rand1 - 1);
        }
      }
    }
    return new_path;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_shortcutter_service");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  PathShortcutter ps;
  ps.run();
  ROS_INFO("Path shortcutter service: ready to check joint configurations");

  ros::waitForShutdown();
  return 0;
}
