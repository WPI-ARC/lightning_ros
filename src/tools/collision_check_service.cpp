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
#include <boost/thread.hpp>

#include "lightning/CollisionCheck.h"
#include "lightning/IntArray.h"
#include "lightning/IntArray2D.h"
#include "lightning/Status.h"
#include "lightning/collision_checker.h"
#include "lightning/collision_utils.h"

/**
 * The CollisionCheckService advertises a CollisionCheck service which takes a
 * set of paths and returns the ranges of indices of invalid sections.
 */
class CollisionCheckService {
 public:
  CollisionCheckService() : private_handle_("~") {
    // Retrieve parameter information from ROS parameter server.
    handle_.getParam("step_size", step_size_);
    handle_.getParam("increment", increment_);
    private_handle_.getParam("num_threads", num_threads_);

    // Wait for the get_planning_scene service to be advertised so that the
    // CollisionChecker can get the initial PlanningScene without any issue.
    ros::service::waitForService("/get_planning_scene");

    // Construct the collision checker with the retrieved step size.
    collision_checker_ = new CollisionChecker(step_size_);
  }

  ~CollisionCheckService() {
    delete collision_checker_;
  }

  // Begin advertising the service.
  void run() {
    service_ = handle_.advertiseService(
        "collision_check", &CollisionCheckService::getAllInvalidSections, this);
  }

  // Callback for CollisionCheck service. req includes a list of paths to check
  // (essentially a vector of vectors of vectors) and the response contains a
  // list of lists of pairs of integers representing the start and end indices
  // of the colliding portions of each path.
  bool getAllInvalidSections(lightning::CollisionCheck::Request &req,
                             lightning::CollisionCheck::Response &res) {
    ros::WallTime start_time = ros::WallTime::now();
    res.status.status = lightning::Status::FAILURE;
    ROS_INFO("Collision check service: received a set of %u paths",
             (unsigned int)req.paths.size());

    all_invalid_sections_.clear();
    all_invalid_sections_.resize(req.paths.size());

    if (collision_checker_->acquireScene(req.group_name)) {
      joint_names_ = collision_checker_->getJointNames();
      num_joints_ = joint_names_.size();
      // get information out of message
      std::vector<std::vector<std::vector<double> > > all_paths;
      std::vector<std::vector<double> > current_path;
      for (unsigned int i = 0; i < req.paths.size(); i++) {
        current_path.clear();
        for (unsigned int j = 0; j < req.paths[i].points.size(); j++) {
          current_path.push_back(req.paths[i].points[j].values);
        }
        all_paths.push_back(current_path);
      }

      // start threads
      std::vector<boost::thread *> all_threads;
      double paths_per_thread = ((double)req.paths.size()) / num_threads_;
      ROS_INFO("Collision check service: paths per thread = %f",
               paths_per_thread);
      // Set of paths for thread to collision check.
      std::vector<std::vector<std::vector<double> > > paths_for_thread;
      // The indicies corresponding to the paths a thread will work on, so that
      // it knows where to store its results in the result vector.
      std::vector<int> indicies_for_thread;
      int threads_used = 0;

      // Iterates through each path and after having gone through the
      // appropriate number of paths, passes the paths off to a new thread to
      // handle the collision checking.
      for (unsigned int i = 0; i < all_paths.size(); i++) {
        paths_for_thread.push_back(all_paths[i]);
        indicies_for_thread.push_back(i);
        if (i + 1 >= (unsigned int)((threads_used + 1) * paths_per_thread)) {
          all_threads.push_back(new boost::thread(
              &CollisionCheckService::i_s_thread_func, this, paths_for_thread,
              indicies_for_thread, threads_used == 0));
          threads_used++;
          paths_for_thread.clear();
          indicies_for_thread.clear();
        }
      }

      // wait for threads to finish and then clean them up
      for (unsigned int i = 0; i < all_threads.size(); i++) {
        all_threads[i]->join();
      }
      for (unsigned int i = 0; i < all_threads.size(); i++) {
        delete all_threads[i];
      }
      all_threads.clear();

      // put invalid sections into message
      lightning::IntArray invalid_section_msg;
      lightning::IntArray2D invalid_sections_msg;
      for (unsigned int i = 0; i < all_invalid_sections_.size(); i++) {
        invalid_sections_msg.points.clear();
        for (unsigned int j = 0; j < all_invalid_sections_[i].size(); j++) {
          invalid_section_msg.values = all_invalid_sections_[i][j];
          invalid_sections_msg.points.push_back(invalid_section_msg);
        }
        res.invalid_sections.push_back(invalid_sections_msg);
      }

      // Be sure to release the scene.
      collision_checker_->releaseScene();
      res.status.status = lightning::Status::SUCCESS;
    }
    ROS_INFO("The CollisionCheckService took %f seconds to run.", (ros::WallTime::now() - start_time).toSec());
    return true;
  }

 private:
  // ROS Node handles.
  ros::NodeHandle private_handle_, handle_;

  // Pointer to collision checker.
  // Should be moved away from a raw pointer at some point.
  CollisionChecker *collision_checker_;

  // Server for the collision_check service.
  ros::ServiceServer service_;

  // Step size to pass to collision_checker. Not actually used by
  // collision_checker_ in the methods which are called here.
  double step_size_;

  // How far to increment along the path when checking points to collision
  // check. A value of 1 means that each point is checked, 2 means every other
  // point is checked, 3 means every third, etc. A larger increment means that
  // this is more likely to miss a collision, but that it is going to run
  // faster.
  // When the path is being collision checked and a the start or end of a
  // colliding interval is encountered, then the checker will check all the
  // points in the vicinity to determine the exact start and end points of the
  // invalid regions.
  int increment_;

  // Maximum number of collision checking threads to run at a time.
  int num_threads_;

  // Information about the paths being collision checked.
  std::vector<std::vector<std::vector<int> > > all_invalid_sections_;
  boost::mutex invalid_sections_lock_;
  std::vector<std::string> joint_names_;
  int num_joints_;

  /**
   * This is the function that should be called when spawning the collision
   *checking threads.
   * It takes a set of paths and stores the indices of their colliding sections
   * in the all_invalid_sections_ member variable.
   *
   * @param paths A vector of paths to check for collisions.
   * @param indices The indicies of all_invalid_sections_ which correspond to
   * each path in paths.
   * @param first Whether or not this is the first thread being created; passed
   * to the CollisionChecker class so that it knows how to handle thread safety.
   * When in doubt, set to false.
   */
  void i_s_thread_func(
      std::vector<std::vector<std::vector<double> > > paths,
      std::vector<int> indicies, bool first) {
    // Initialize variables.
    std::vector<std::vector<int> > invalid_sections_for_path;
    std::vector<int> current_invalid_section;
    current_invalid_section.resize(2);
    int start_invalid = -1;
    bool tracking_invalid = false;

    // Iterate through each point in each path and determine whether it is in
    // collision or not, and generate a list of pairs of points which correspond
    // to the starts and ends of invalid sets of points in the paths.
    for (unsigned int i = 0; i < paths.size(); i++) {
      ROS_INFO("Collision check service: working on path %i", indicies[i]);
      invalid_sections_for_path.clear();
      start_invalid = -1;
      tracking_invalid = false;
      // Iterate through each point in the path.
      for (unsigned int j = 0; j < paths[i].size(); j += increment_) {
        if (collision_checker_->isStateValid(paths[i][j],
                                             !first /*Thread safety*/)) {
          // Checks for if we are at the end of an invalid section.
          if (tracking_invalid) {
            current_invalid_section[0] = start_invalid;
            current_invalid_section[1] = j;
            invalid_sections_for_path.push_back(current_invalid_section);
            tracking_invalid = false;
            if (increment_ > 1 && j > 0) {
              for (int k = 1; k < increment_; k++) {
                if (collision_checker_->isStateValid(paths[i][j - k], !first)) {
                  current_invalid_section[1] -= 1;
                }
                else break;
              }
            }
          }
        } else {
          // Debugging info.
          if (j == paths[i].size() - 1) {
            ROS_INFO("Collision check service: goal state is invalid");
          } else {
            ROS_INFO("State %d invalid.", j);
          }

          // Checks if we are at the start of an invalid section.
          if (!tracking_invalid) {
            start_invalid = j - 1;
            tracking_invalid = true;
            // Check the state before this one to see if it is in collision.
            // This is necessary due to the nature of the incrementing.
            if (increment_ > 1 && j > 0) {
              for (int k = 1; k < increment_; k++) {
                if (!collision_checker_->isStateValid(paths[i][j - k], !first)) {
                  start_invalid -= 1;
                }
                else break;
              }
            }
          }
        }
      }
      if (tracking_invalid) {  // still tracking an invalid section outside of
                               // loop, so goal is invalid
        current_invalid_section[0] = start_invalid;
        current_invalid_section[1] = paths[i].size();
        invalid_sections_for_path.push_back(current_invalid_section);
      }

      // store invalid sections to return
      invalid_sections_lock_.lock();
      all_invalid_sections_[indicies[i]] = invalid_sections_for_path;
      invalid_sections_lock_.unlock();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "collision_check_service");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("Starting up collision_check_service.");
  CollisionCheckService cc;
  cc.run();
  ROS_INFO("Collision check service: ready to check joint configurations");

  ros::waitForShutdown();
  return 0;
}
