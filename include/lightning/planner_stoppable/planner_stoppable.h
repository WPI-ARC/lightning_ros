#ifndef PLANNER_STOPPABLE_H_
#define PLANNER_STOPPABLE_H_

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetMotionPlan.h>

// Local messages.
#include "lightning/StopPlanning.h"
#include "lightning/Status.h"

class PlannerStoppable {
 public:
  /**
   * Constructor for PlannerStoppable.
   *
   * @arg name is a planner name (for debugging).
   * @arg stop_name is the topic name for stopping the planner.
   */
  PlannerStoppable(std::string name, std::string stop_name);

  ~PlannerStoppable() {}

  /**
   * Start running the planner.
   */
  void run();

 private:
  // Just need to implement stop_planning and computePlan.
  // modification: callback for messages for stopping planning
  void stop_planning(const lightning::StopPlanning::ConstPtr &msg);

  /**
    @brief Planning - choose the correct planner and then call it
    with the request.
    */
  bool computePlan(moveit_msgs::GetMotionPlan::Request &request,
                   moveit_msgs::GetMotionPlan::Response &response);

  bool initialize();

  // Service for calling computePlan.
  ros::ServiceServer plan_path_service_;
  // Public facing node handle.
  ros::NodeHandle node_handle_;

  planning_scene::PlanningScenePtr ps_;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  planning_interface::PlannerManagerPtr planner_instance_;

  // Private node handle for stopping.
  ros::NodeHandle stop_nh_;
  // Corresponding subscriber.
  ros::Subscriber stop_planning_subscriber_;

  // planner_name (debugging) and stop_name for subscribing to a stop task.
  std::string stop_name_, planner_name_;
};

#endif  // PLANNER_STOPPABLE_H_
