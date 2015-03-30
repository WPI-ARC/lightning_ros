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

class CollisionCheckService {
 public:
  CollisionCheckService() : private_handle_("~") {
    handle_.getParam("step_size", step_size_);
    private_handle_.getParam("num_threads", num_threads_);
    ros::service::waitForService("get_planning_scene");
    ROS_INFO("About to construct Collision Checker.");
    collision_checker_ = new CollisionChecker(step_size_);
    ROS_INFO("Constructed Collision Checker.");
  }

  ~CollisionCheckService() {
    delete collision_checker_;
  }

  void run() {
    if (!collision_checker_->collisionModelsInterfaceLoadedModels()) {
      ROS_ERROR("Collision check service: Collision models not loaded");
    } else {
      service_ = handle_.advertiseService(
          "collision_check", &CollisionCheckService::getAllInvalidSections,
          this);
    }
  }

  bool getAllInvalidSections(lightning::CollisionCheck::Request &req,
                             lightning::CollisionCheck::Response &res) {
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
      std::vector<std::vector<std::vector<double> > > paths_for_thread;
      std::vector<int> indicies_for_thread;
      int threads_used = 0;
      for (unsigned int i = 0; i < all_paths.size(); i++) {
        paths_for_thread.push_back(all_paths[i]);
        indicies_for_thread.push_back(i);
        if (i + 1 >= (unsigned int)((threads_used + 1) * paths_per_thread)) {
          // TODO: Check thread safety of i_s_thread_func_first.
          all_threads.push_back(
              new boost::thread(&CollisionCheckService::i_s_thread_func_first,
                                this, paths_for_thread, indicies_for_thread));
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

      // put invalid sections back into message
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
      collision_checker_->releaseScene();
      res.status.status = lightning::Status::SUCCESS;
    }
    return true;
  }

 private:
  ros::NodeHandle private_handle_, handle_;
  CollisionChecker *collision_checker_;
  ros::ServiceServer service_;
  double step_size_;
  int num_threads_;
  std::vector<std::vector<std::vector<int> > > all_invalid_sections_;
  boost::mutex invalid_sections_lock_;
  std::vector<std::string> joint_names_;
  int num_joints_;

  void i_s_thread_func_first(
      std::vector<std::vector<std::vector<double> > > paths,
      std::vector<int> indicies) {
    std::vector<std::vector<int> > invalid_sections_for_path;
    std::vector<int> current_invalid_section;
    current_invalid_section.resize(2);
    int start_invalid = -1;
    bool tracking_invalid = false;
    for (unsigned int i = 0; i < paths.size(); i++) {
      ROS_INFO("Collision check service: working on path %i", indicies[i]);
      invalid_sections_for_path.clear();
      start_invalid = -1;
      tracking_invalid = false;
      for (unsigned int j = 0; j < paths[i].size(); j++) {
        if (collision_checker_->isStateValid(paths[i][j])) {
          if (tracking_invalid) {
            current_invalid_section[0] = start_invalid;
            current_invalid_section[1] = j;
            invalid_sections_for_path.push_back(current_invalid_section);
            tracking_invalid = false;
          }
        } else {
          if (j == paths[i].size() - 1) {
            ROS_INFO("Collision check service: goal state is invalid");
          }
          if (!tracking_invalid) {
            start_invalid = j - 1;
            tracking_invalid = true;
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
