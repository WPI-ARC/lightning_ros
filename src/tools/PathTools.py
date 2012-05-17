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

import roslib
roslib.load_manifest("lightning")
import rospy

import threading
import sys

from lightning.msg import Float64Array, Float64Array2D, DrawPoints
from lightning.srv import CollisionCheck, CollisionCheckRequest, PathShortcut, PathShortcutRequest
from arm_navigation_msgs.srv import GetMotionPlan, GetMotionPlanRequest
from arm_navigation_msgs.msg import JointConstraint

COLLISION_CHECK = "collision_check"
SHORTCUT_PATH_NAME = "shortcut_path"
DISPLAY_POINTS = "draw_points"
PLANNER_NAME = "plan_kinematic_path"

class PlanTrajectoryWrapper:

    def __init__(self, node_type, num_planners=1):
        self.planners = ["/%s_planner_node%i/%s" % (node_type, i, PLANNER_NAME) for i in xrange(num_planners)]
        rospy.loginfo("Initializaing %i planners for %s" % (num_planners, node_type))
        self.planners_available = [True for i in xrange(num_planners)]
        self.planner_lock = threading.Lock()
        self.released_event = threading.Event()
        self.released_event.set()

    #need to call acquire_planner before calling plan_trajectory
    def acquire_planner(self):
        planner_number = self._wait_for_planner()
        while planner_number == -1:
            self.released_event.wait()
            planner_number = self._wait_for_planner()
        return planner_number
   
    #need to call release_planner after done calling plan_trajectory
    def release_planner(self, index):
        self.planner_lock.acquire()
        self.planners_available[index] = True
        self.released_event.set()
        self.planner_lock.release()

    def _wait_for_planner(self):
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
    def plan_trajectory(self, start_point, goal_point, planner_number, joint_names, group_name, planning_time, planner_config_name="RRTConnectkConfig1"):
        planner_client = rospy.ServiceProxy(self.planners[planner_number], GetMotionPlan)
        rospy.loginfo("Plan Trajectory Wrapper: got a plan_trajectory request for %s with start = %s and goal = %s" % (self.planners[planner_number], start_point, goal_point))
        req = GetMotionPlanRequest()
        req.motion_plan_request.workspace_parameters.workspace_region_pose.header.stamp = rospy.get_rostime()
        req.motion_plan_request.group_name = group_name
        req.motion_plan_request.num_planning_attempts = 1
        req.motion_plan_request.allowed_planning_time = rospy.Duration(planning_time)
        req.motion_plan_request.planner_id = planner_config_name #using RRT planner by default

        req.motion_plan_request.start_state.joint_state.header.stamp = rospy.get_rostime()
        req.motion_plan_request.start_state.joint_state.name = joint_names
        req.motion_plan_request.start_state.joint_state.position = start_point

        req.motion_plan_request.goal_constraints.joint_constraints = []
        for i in xrange(len(joint_names)):
            temp_constraint = JointConstraint()
            temp_constraint.joint_name = joint_names[i]
            temp_constraint.position = goal_point[i]
            req.motion_plan_request.goal_constraints.joint_constraints.append(temp_constraint)

        #call the planner
        rospy.wait_for_service(self.planners[planner_number])
        rospy.loginfo("Plan Trajectory Wrapper: sent request to service %s" % planner_client.resolved_name)
        try:
            response = planner_client(req)
        except rospy.ServiceException, e:
            rospy.loginfo("Plan Trajectory Wrapper: service call failed: %s"%e)
            return None

        rospy.loginfo("Plan Trajectory Wrapper: %s returned" % (self.planners[planner_number]))
        if response.error_code.val == response.error_code.SUCCESS:
            return [pt.positions for pt in response.trajectory.joint_trajectory.points]
        else:
            rospy.loginfo("Plan Trajectory Wrapper: service call to %s was unsuccessful" % planner_client.resolved_name)
            return None

class ShortcutPathWrapper:

    def shortcut_path(self, original_path, group_name):
        shortcut_path_client = rospy.ServiceProxy(SHORTCUT_PATH_NAME, PathShortcut)
        shortcut_req = PathShortcutRequest()
        shortcut_req.path = [Float64Array(p) for p in original_path]
        shortcut_req.group_name = group_name
        rospy.wait_for_service(SHORTCUT_PATH_NAME)
        response = shortcut_path_client(shortcut_req)
        return [p.values for p in response.new_path]

class InvalidSectionWrapper:

    def get_invalid_sections_for_path(self, original_path, group_name):
        section = self.get_invalid_sections_for_paths([original_path], group_name)
        if len(section) > 0:
            return section[0]
        else:
            return None

    def get_invalid_sections_for_paths(self, orig_paths, group_name):
        collision_check_client = rospy.ServiceProxy(COLLISION_CHECK, CollisionCheck)
        cc_req = CollisionCheckRequest();
        cc_req.paths = [Float64Array2D([Float64Array(point) for point in path]) for path in orig_paths];
        cc_req.group_name = group_name
        rospy.loginfo("Plan Trajectory Wrapper: sending request to collision checker")
        rospy.wait_for_service(COLLISION_CHECK)
        response = collision_check_client(cc_req);
        return [[sec.values for sec in individualPathSections.points] for individualPathSections in response.invalid_sections];

class DrawPointsWrapper:

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
        self.display_points_publisher = rospy.Publisher(DISPLAY_POINTS, DrawPoints)

    def draw_points(self, path, model_group_name, point_group_name, point_type, rgb, display_density, point_radius=0.03):
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
        draw_message = DrawPoints()
        draw_message.action = draw_message.ACTION_CLEAR
        self.display_points_publisher.publish(draw_message)

if __name__ == "__main__":
    if len(sys.argv) == 8:
        isw = InvalidSectionWrapper()
        path = [float(sys.argv[i]) for i in xrange(1, len(sys.argv))]
        print isw.get_invalid_sections_for_path([path])
