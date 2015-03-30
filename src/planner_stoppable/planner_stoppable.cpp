#include "lightning/planner_stoppable/planner_stoppable.h"

PlannerStoppable::PlannerStoppable(std::string name, std::string stop_name)
    : node_handle_("~") {
  planner_name_ = name;
  stop_name_ = stop_name;
  ROS_INFO("PlannerStoppable started.");
}

void PlannerStoppable::run() {
  ros::service::waitForService("get_planning_scene");

  // Start monitor for keeping planningscene up-to-date.

  // Initializes planner and the such.
  if (!initialize()) return;

  // Advertise various services.
  plan_path_service_ = node_handle_.advertiseService(
      "plan_kinematic_path", &PlannerStoppable::computePlan, this);

  stop_planning_subscriber_ =
      stop_nh_.subscribe(stop_name_, 10, &PlannerStoppable::stop_planning, this);

  ROS_INFO("Listening on %s and %s for ompl ros interface",
           planner_name_.c_str(), stop_name_.c_str());
}

/**
 * intialize() handles retrieving the planner name from the parameter server and
 * initializing the PlannerManager.
 * The code is largely copied from the moveit tutorials.
 * @returns success
 */
bool PlannerStoppable::initialize() {
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader("/robot_description"));
  robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();
  ps_.reset(new planning_scene::PlanningScene(robot_model));
  // XXX: ps_ may not be thread-safe.
  monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ps_, robot_model_loader));

  // TODO: allow overriding of default service and topic subscriptions for
  // monitor_.
  // Get initial planning scene state from /get_planning_scene service.
  monitor_->requestPlanningSceneState("get_planning_scene");
  monitor_->startSceneMonitor("move_group/monitored_planning_scene"); // Default "/planning_scene"

  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> >
      planner_plugin_loader;
  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS param server, and then load the planner
  // making sure to catch all exceptions.
  if (!node_handle_.getParam("default_planner_config", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try {
    planner_plugin_loader.reset(
        new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
  } catch (pluginlib::PluginlibException& ex) {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader "
                     << ex.what());
    return false; // Although I assume we never reach this...
  }
  try {
    planner_instance_.reset(
        planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance_->initialize(robot_model, node_handle_.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '"
                    << planner_instance_->getDescription() << "'");
  } catch (pluginlib::PluginlibException& ex) {
    // Just debugging/logging info.
    const std::vector<std::string>& classes =
        planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i) ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '"
                     << planner_plugin_name << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
    return false;
  }

  return true;
}

void PlannerStoppable::stop_planning(const lightning::StopPlanning::ConstPtr &msg) {
  // Note: If we want to be able to provide an error code, terminate() does
  // return success (as a bool).
  planner_instance_->terminate();
}

bool PlannerStoppable::computePlan(moveit_msgs::GetMotionPlan::Request &request,
                   moveit_msgs::GetMotionPlan::Response &response) {
  // Temporary response variable (separate from message).
  planning_interface::MotionPlanResponse res;
  planning_interface::PlanningContextPtr context =
      planner_instance_->getPlanningContext(
          ps_, request.motion_plan_request,
          res.error_code_);

  // Blocks until it finishes.
  context->solve(res);

  res.getMessage(response.motion_plan_response);

  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    return false;
  }
  return true;
}
