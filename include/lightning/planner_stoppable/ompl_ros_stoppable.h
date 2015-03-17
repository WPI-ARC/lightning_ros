/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Sachin Chitta*/

/* This is a modification of ompl_ros.h from the ompl_ros_interface
 * package created by the above organization/author.
 *
 * The modification was made by Authors: Cameron Lee (cameronlee@berkeley.edu)
 * and Dmitry Berenson (berenson@eecs.berkeley.edu)
 * 2012, University of California, Berkeley
 */

#ifndef OMPL_ROS_STOPPABLE_H_
#define OMPL_ROS_STOPPABLE_H_

// ROS
#include <ros/console.h>
#include <ros/ros.h>

// Planning environment and models
#include <planning_environment/models/collision_models_interface.h>

// OMPL ROS Interface
// modification: remove rpy_ik_task_space_planner and added planner_stoppable
// includes
//#include <ompl_ros_interface/planners/ompl_ros_rpy_ik_task_space_planner.h>
#include "lightning/planner_stoppable/ompl_ros_stoppable_planning_group.h"
#include "lightning/planner_stoppable/ompl_ros_stoppable_joint_planner.h"
#include <ompl_ros_interface/helpers/ompl_ros_conversions.h>

// Diagnostics Message
#include <ompl_ros_interface/OmplPlannerDiagnostics.h>

// modification
#include "lightning/StopPlanning.h"
#include "lightning/Status.h"

/**
 * @class OmplRosStoppable
 * @brief Initializes a bunch of planners for different groups (collections of
 * joints and links, e.g. a robot arm). This class gets all its parameters from
 * the ROS parameter server. After initializing, just call run on the class to
 * start it running.
 */
class OmplRosStoppable {
 public:
  // modification: commented out a constructor and added a new one
  // OmplRosStoppable();

  // planner name (currently used for debugging) and topic name to use for
  // stopping the planner
  OmplRosStoppable(std::string name, std::string stop_name);
  ~OmplRosStoppable();

  /**
    @brief Start running the planner
    */
  void run(void);

  /**
    @brief Get a particular planner for a given group
    */
  boost::shared_ptr<OmplRosStoppablePlanningGroup> &getPlanner(
      const std::string &group_name, const std::string &planner_config_name);

 private:
  // modification: callback for messages for stopping planning
  void stop_planning(const lightning::StopPlanning::ConstPtr &msg);

  /**
    @brief Planning - choose the correct planner and then call it
    with the request.
    */
  bool computePlan(arm_navigation_msgs::GetMotionPlan::Request &request,
                   arm_navigation_msgs::GetMotionPlan::Response &response);

  /**
    @brief Get the names of all groups we will be planning for
    */
  bool getGroupNamesFromParamServer(const std::string &param_server_prefix,
                                    std::vector<std::string> &group_names);

  bool initialize(const std::string &param_server_prefix);

  bool initializePlanningMap(const std::string &param_server_prefix,
                             const std::vector<std::string> &group_names);

  bool initializePlanningInstance(const std::string &param_server_prefix,
                                  const std::string &group_name,
                                  const std::string &planner_config_name);
  /**
    @brief Map from planner name[group name] to an instance of
    ompl_ros_interface::OmplPlanningGroup
    */
  std::map<std::string, boost::shared_ptr<OmplRosStoppablePlanningGroup> >
      planner_map_;

  // ROS interface
  boost::shared_ptr<OmplRosStoppablePlanningGroup> empty_ptr;
  ros::ServiceServer plan_path_service_;
  planning_environment::CollisionModelsInterface *collision_models_interface_;
  ros::NodeHandle node_handle_;

  // modification: private node handle
  ros::NodeHandle nh_;

  std::string default_planner_config_;
  bool publish_diagnostics_;
  ros::Publisher diagnostic_publisher_;

  // modification: topic name for messages to stop planning and planner name
  // (currently used for debugging)
  std::string stop_name_, planner_name_;
  ros::Subscriber stop_planning_subscriber_;
};
#endif  // OMPL_ROS_STOPPABLE_H_
