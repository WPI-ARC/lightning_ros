"""
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
"""

"""
This file contains several wrapper classes for planning paths, performing
  collision checking, retrieving paths, and drawing points in RViz.
None of these classes really perform actualy work but rather are just calling
  ROS services.
"""

import roslib
import rospy

import threading
import sys

from lightning.msg import Float64Array, Float64Array2D, DrawPoints
from lightning.srv import CollisionCheck, CollisionCheckRequest, PathShortcut, PathShortcutRequest
from moveit_msgs.srv import GetMotionPlan, GetMotionPlanRequest
from moveit_msgs.msg import JointConstraint, Constraints

# Names of Topics/Services to be advertised/used by these wrappers.
# The name of the collision checking service.
COLLISION_CHECK = "collision_check"
# The name of the path shortcutting service.
SHORTCUT_PATH_NAME = "shortcut_path"
# Topic to publish to for drawing points in RViz.
DISPLAY_POINTS = "draw_points"
# Name of the planner_stoppable services advertised from various lightning nodes.
PLANNER_NAME = "plan_kinematic_path"

class PlanTrajectoryWrapper:
    """
      This wrapper class handles calling the GetMotionPlan service of
        planner_stoppable type nodes, handling keeping track of multiple
        planners (for multi-threading), constructing the service requests and
        extracting the useful information from the response.
    """

    def __init__(self, node_type, num_planners=1):
        """
          Constructor for PlanTrajectoryWrapper.

          Args:
            node_type (string): The type of planner that this is being used by,
              generally "pfs" or "rr".
            num_planners (int): The number of planner nodes that are being used.
        """
        self.planners = ["%s_planner_node%i/%s" % (node_type, i, PLANNER_NAME) for i in xrange(num_planners)]
        rospy.loginfo("Initializaing %i planners for %s" % (num_planners, node_type))
        self.planners_available = [True for i in xrange(num_planners)]
        self.planner_lock = threading.Lock()
        self.released_event = threading.Event()
        self.released_event.set()

    def acquire_planner(self):
        """
          Acquires a planner lock so that plan_trajectory() can be called.
          This must be called before calling plan_trajectory().

          Returns:
            int: The index of the planner whose lock was acquired.
              This is only really relevant if multiple planners are being used
              and is the number that should be passed as the planner_number
              to plan_trajectory().
        """
        planner_number = self._wait_for_planner()
        while planner_number == -1:
            self.released_event.wait()
            planner_number = self._wait_for_planner()
        return planner_number

    def release_planner(self, index):
        """
          Releases the planner lock that you acquired so that plan_trajectory()
            can be called on that planner by someone else.
          This should be called after you are done calling plan_trajectory().
        """
        self.planner_lock.acquire()
        self.planners_available[index] = True
        self.released_event.set()
        self.planner_lock.release()

    def _wait_for_planner(self):
        """
          Waits for at least one planner lock to release so that it can be
            acquired.
        """
        self.planner_lock.acquire()
        acquired_planner = -1
        for i, val in enumerate(self.planners_available):
            if val:
                self.planners_available[i] = False
                if not any(self.planners_available):
                    self.released_event.clear()
                acquired_planner = i
                break
        self.planner_lock.release()
        return acquired_planner

    #planner to get new trajectory from start_point to goal_point
    #planner_number is the number received from acquire_planner
    def plan_trajectory(self, start_point, goal_point, planner_number, joint_names, group_name, planning_time, planner_config_name):
        """
          Given a start and goal point, returns the planned path.

          Args:
            start_point (list of float): A starting joint configuration.
            goal_point (list of float): A goal joint configuration.
            planner_number (int): The index of the planner to be used as
              returned by acquire_planner().
            joint_names (list of str): The name of the joints corresponding to
              start_point and goal_point.
            group_name (str): The name of the group to which the joint names
              correspond.
            planning_time (float): Maximum allowed time for planning, in seconds.
            planner_config_name (str): Type of planner to use.
          Return:
            list of list of float: A sequence of points representing the joint
              configurations at each point on the path.
        """
        planner_client = rospy.ServiceProxy(self.planners[planner_number], GetMotionPlan)
        rospy.loginfo("Plan Trajectory Wrapper: got a plan_trajectory request for %s with start = %s and goal = %s" % (self.planners[planner_number], start_point, goal_point))
        # Put together the service request.
        req = GetMotionPlanRequest()
        req.motion_plan_request.workspace_parameters.header.stamp = rospy.get_rostime()
        req.motion_plan_request.group_name = group_name
        req.motion_plan_request.num_planning_attempts = 1
        req.motion_plan_request.allowed_planning_time = planning_time
        req.motion_plan_request.planner_id = planner_config_name #using RRT planner by default

        req.motion_plan_request.start_state.joint_state.header.stamp = rospy.get_rostime()
        req.motion_plan_request.start_state.joint_state.name = joint_names
        req.motion_plan_request.start_state.joint_state.position = start_point

        req.motion_plan_request.goal_constraints.append(Constraints())
        req.motion_plan_request.goal_constraints[0].joint_constraints = []
        for i in xrange(len(joint_names)):
            temp_constraint = JointConstraint()
            temp_constraint.joint_name = joint_names[i]
            temp_constraint.position = goal_point[i]
            temp_constraint.tolerance_above = 0.05;
            temp_constraint.tolerance_below = 0.05;
            req.motion_plan_request.goal_constraints[0].joint_constraints.append(temp_constraint)

        #call the planner
        rospy.wait_for_service(self.planners[planner_number])
        rospy.loginfo("Plan Trajectory Wrapper: sent request to service %s" % planner_client.resolved_name)
        try:
            response = planner_client(req)
        except rospy.ServiceException, e:
            rospy.loginfo("Plan Trajectory Wrapper: service call failed: %s"%e)
            return None

        # Pull a list of joint positions out of the returned plan.
        rospy.loginfo("Plan Trajectory Wrapper: %s returned" % (self.planners[planner_number]))
        if response.motion_plan_response.error_code.val == response.motion_plan_response.error_code.SUCCESS:
            return [pt.positions for pt in response.motion_plan_response.trajectory.joint_trajectory.points]
        else:
            rospy.loginfo("Plan Trajectory Wrapper: service call to %s was unsuccessful" % planner_client.resolved_name)
            return None

class ShortcutPathWrapper:
    """
      This is a very thin wrapper over the path shortcutting service.
    """

    def shortcut_path(self, original_path, group_name):
        """
          Shortcuts a path, where the path is for a given group name.

          Args:
            original_path (list of list of float): The path, represented by
              a list of individual joint configurations.
            group_name (str): The group for which the path was created.

          Return:
            list of list of float: The shortcutted version of the path.
        """
        shortcut_path_client = rospy.ServiceProxy(SHORTCUT_PATH_NAME, PathShortcut)
        shortcut_req = PathShortcutRequest()
        shortcut_req.path = [Float64Array(p) for p in original_path]
        shortcut_req.group_name = group_name
        rospy.wait_for_service(SHORTCUT_PATH_NAME)
        response = shortcut_path_client(shortcut_req)
        return [p.values for p in response.new_path]

class InvalidSectionWrapper:
    """
      This is a very thin wrapper over the collision checking service.
    """

    def get_invalid_sections_for_path(self, original_path, group_name):
        """
          Returns the invalid sections for a single path.

          Args:
            original_path (list of list of float): The path to collision check,
              represnted by a list of individual joint configurations.
            group_name (str): The joint group for which the path was created.

          Return:
            list of pairs of indicies, where each index in a pair is the start
              and end of an invalid section.
        """
        section = self.get_invalid_sections_for_paths([original_path], group_name)
        if len(section) > 0:
            return section[0]
        else:
            return None

    def get_invalid_sections_for_paths(self, orig_paths, group_name):
        """
          Returns the invalid sections for a set of paths.

          Args:
            orig_paths (list of paths): The paths to collision check,
              represnted by a list of individual joint configurations.
            group_name (str): The joint group for which the paths were created.

          Return:
            list of list of pairs of indicies, where each index in a pair is the
              start and end of an invalid section.
        """
        collision_check_client = rospy.ServiceProxy(COLLISION_CHECK, CollisionCheck)
        cc_req = CollisionCheckRequest();
        cc_req.paths = [Float64Array2D([Float64Array(point) for point in path]) for path in orig_paths];
        cc_req.group_name = group_name
        rospy.loginfo("Plan Trajectory Wrapper: sending request to collision checker")
        rospy.wait_for_service(COLLISION_CHECK)
        response = collision_check_client(cc_req);
        return [[sec.values for sec in individualPathSections.points] for individualPathSections in response.invalid_sections];

class DrawPointsWrapper:
    """
      Wrapper to draw all the points for a path in RVIz.
      The points are drawn on the DISPLAY_POINTS topic, which is subscribed to
      by PointDrawer.py. This class is used when running tests.
    """

    #point colors
    WHITE = (1.0, 1.0, 1.0)
    BLACK = (0.0, 0.0, 0.0)
    RED = (1.0, 0.0, 0.0)
    GREEN = (0.0, 1.0, 0.0)
    BLUE = (0.0, 0.0, 1.0)
    MAGENTA = (1.0, 0.0, 1.0)
    YELLOW = (1.0, 1.0, 0.0)
    GREENBLUE = (0.0, 1.0, 1.0)

    #point types
    ANGLES = "angles"
    POSES = "poses"

    def __init__(self):
        self.display_points_publisher = rospy.Publisher(DISPLAY_POINTS, DrawPoints, queue_size=10)

    def draw_points(self, path, model_group_name, point_group_name, point_type, rgb, display_density, point_radius=0.03):
        """
          Draws the points of a given path in RViz.

          Args:
            path (list of list of float): The path to draw.
            model_group_name (str): The name of the joint group in question.
              For the PR2 arms, this would be "right_arm" or "left_arm".
            point_group_name (str): The namespace under which the points will
              show up in RViz.
            point_type (str): Type of point, ANGLES or POSES.
            rgb (tuple of float): Color of points being drawn. Some colors are
              pre-defined as members of this class.
            display_density (float): The fraction of the path to be displayed.
              For instance, if display_density = 0.25, one in four points will
              be shown.
            point_radius (float): Size of the individual points to be drawn.
        """
        draw_message = DrawPoints()
        draw_message.points = [Float64Array(p) for p in path]
        draw_message.model_group_name = model_group_name
        draw_message.point_group_name = point_group_name
        draw_message.point_type = draw_message.POINT_TYPE_ANGLES if point_type == DrawPointsWrapper.ANGLES else draw_message.POINT_TYPE_POSES
        draw_message.display_density = display_density
        draw_message.red, draw_message.green, draw_message.blue = rgb
        draw_message.action = draw_message.ACTION_ADD
        draw_message.point_radius = point_radius
        self.display_points_publisher.publish(draw_message)

    def clear_points(self):
        """
          Clears all of the points from the display.
        """
        draw_message = DrawPoints()
        draw_message.action = draw_message.ACTION_CLEAR
        self.display_points_publisher.publish(draw_message)

if __name__ == "__main__":
    if len(sys.argv) == 8:
        isw = InvalidSectionWrapper()
        path = [float(sys.argv[i]) for i in xrange(1, len(sys.argv))]
        print isw.get_invalid_sections_for_path([path])
