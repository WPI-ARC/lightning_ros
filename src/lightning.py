#!/usr/bin/env python
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
roslib.load_manifest("lightning");
import rospy
import actionlib
import threading

from lightning.msg import RRAction, RRGoal, PFSAction, PFSGoal, Float64Array, StopPlanning
from lightning.srv import ManagePathLibrary, ManagePathLibraryRequest, PathShortcut, PathShortcutRequest
from arm_navigation_msgs.srv import GetMotionPlan, GetMotionPlanResponse
from tools.PathTools import ShortcutPathWrapper, DrawPointsWrapper
from trajectory_msgs.msg import JointTrajectoryPoint

PLANNER_CONFIG_NAME = "RRTConnectkConfig1"
RR_NODE_NAME = "rr_node"
PFS_NODE_NAME = "pfs_node"
STOP_RR_NAME = "stop_all_rr"
STOP_PFS_NAME = "stop_all_pfs"
LIGHTNING_SERVICE = "lightning_get_path"
SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/get_planning_scene";
MANAGE_LIBRARY = "manage_path_library"

class Lightning:
    def __init__(self):
        rospy.wait_for_service(SET_PLANNING_SCENE_DIFF_NAME); #make sure the environment server is ready before starting up
        self.rr_client = actionlib.SimpleActionClient(RR_NODE_NAME, RRAction)
        self.pfs_client = actionlib.SimpleActionClient(PFS_NODE_NAME, PFSAction)
        self.manage_library_client = rospy.ServiceProxy(MANAGE_LIBRARY, ManagePathLibrary)
        self.store_paths = rospy.get_param("~store_paths")
        self.use_rr = rospy.get_param("~use_RR")
        self.use_pfs = rospy.get_param("~use_PFS")
        if not self.use_rr and not self.use_pfs:
            rospy.logerr("Lightning: at least one of use_RR and use_PFS need to be true")
        self.shortcut_path_wrapper = ShortcutPathWrapper()
        self.lightning_response = None
        self.lightning_service = rospy.Service(LIGHTNING_SERVICE, GetMotionPlan, self.run)
        self.current_joint_names = []
        self.current_group_name = ""
        self.robot_name = rospy.get_param("robot_name")
        self.stop_rr_publisher = rospy.Publisher(STOP_RR_NAME, StopPlanning)
        self.stop_pfs_publisher = rospy.Publisher(STOP_PFS_NAME, StopPlanning)
        self.rr_returned, self.pfs_returned = False, False
        self.done_lock = threading.Lock()
        self.lightning_response_ready_event = threading.Event()

        #if draw_points is True, then display points in rviz
        self.draw_points = rospy.get_param("draw_points")
        if self.draw_points:
            self.draw_points_wrapper = DrawPointsWrapper()

    def _lightning_timeout(self, time):
        self.lightning_response_ready_event.wait(time)
        if self.lightning_response is None: #timeout
            if self.use_rr:
                self._send_stop_rr_planning()
            if self.use_pfs:
                self._send_stop_pfs_planning()

    def run(self, request):
        #make sure the request is valid
        start_and_goal = self._is_valid_motion_plan_request(request)
        if start_and_goal is None:
            response = GetMotionPlanResponse()
            response.error_code.val = response.error_code.PLANNING_FAILED
            return response
        s, g = start_and_goal

        self.rr_returned, self.pfs_returned = False, False
        self.lightning_response = None
        self.lightning_response_ready_event.clear()
        self.current_joint_names = request.motion_plan_request.start_state.joint_state.name
        self.current_group_name = request.motion_plan_request.group_name

        if self.draw_points:
            self.draw_points_wrapper.clear_points()

        #start a timer that stops planners if they take too long
        self.timer = threading.Thread(target=self._lightning_timeout, args=(request.motion_plan_request.allowed_planning_time.to_sec()))

        if self.use_rr:
            rr_client_goal = RRGoal()
            rr_client_goal.start = s
            rr_client_goal.goal = g
            rr_client_goal.joint_names = self.current_joint_names
            rr_client_goal.group_name = self.current_group_name
            rr_client_goal.allowed_planning_time = request.motion_plan_request.allowed_planning_time
            self.rr_client.wait_for_server()
            rospy.loginfo("Lightning: Sending goal to RR")
            self.rr_client.send_goal(rr_client_goal, done_cb=self._rr_done_cb)

        if self.use_pfs:
            pfs_client_goal = PFSGoal()
            pfs_client_goal.start = s
            pfs_client_goal.goal = g
            pfs_client_goal.joint_names = self.current_joint_names
            pfs_client_goal.group_name = self.current_group_name
            pfs_client_goal.allowed_planning_time = request.motion_plan_request.allowed_planning_time
            self.pfs_client.wait_for_server()
            rospy.loginfo("Lightning: Sending goal to PFS")
            self.pfs_client.send_goal(pfs_client_goal, done_cb=self._pfs_done_cb)
        
        self.lightning_response_ready_event.wait()
        if self.lightning_response.error_code.val != self.lightning_response.error_code.SUCCESS:
            rospy.loginfo("Lightning: did not find a path")
        else:
            rospy.loginfo("Lightning: Lightning is responding with a path")
        
        return self.lightning_response

    def _print_error(self, msg):
        rospy.logerr("***ERROR*** %s ***ERROR***" % (msg))

    def _is_valid_motion_plan_request(self, request):
        if request.motion_plan_request.allowed_planning_time.to_sec() <= 0:
            self._print_error("Lightning: requires allowed_planning_time to be greater than 0")
            return None
        
        if len(request.motion_plan_request.goal_constraints.position_constraints) > 0:
            self._print_error("Lightning: does not handle position constraints")
            return None

        s = list(request.motion_plan_request.start_state.joint_state.position)
        g = []
        for jc in request.motion_plan_request.goal_constraints.joint_constraints:
            if jc.tolerance_above != 0 or jc.tolerance_below != 0:
                self._print_error("Lightning: does not handle tolerances")
                return None
            else:
                g.append(jc.position)

        if len(s) == 0:
            self._print_error("Lightning: did not receive a start state")
            return None

        if len(g) == 0:
            self._print_error("Lightning: did not receive a goal state")
            return None
        return s, g

    def _rr_done_cb(self, state, result):
        self.done_lock.acquire()
        self.rr_returned = True
        if result.status.status == result.status.SUCCESS:
            if not self.pfs_returned or self.lightning_response is None:
                rospy.loginfo("Lightning: Got a path from RR")

                self._send_stop_pfs_planning()

                rr_path = [p.values for p in result.repaired_path]
                shortcut = self.shortcut_path_wrapper.shortcut_path(rr_path, self.current_group_name)

                self.lightning_response = self._create_get_motion_plan_response(shortcut)
                self.lightning_response_ready_event.set()
                
                self.done_lock.release()
                
                #display new path in rviz
                if self.draw_points:
                    self.draw_points_wrapper.draw_points(shortcut, self.current_group_name, "final", DrawPointsWrapper.ANGLES, DrawPointsWrapper.GREEN, 0.1)

                if self.store_paths:
                    self._store_path(shortcut, rr_path)
                return
        else:
            rospy.loginfo("Lightning: Call to RR did not return a path")
            if not self.use_pfs or (self.pfs_returned and self.lightning_response is None):
                self.lightning_response = GetMotionPlanResponse()
                self.lightning_response.error_code.val = self.lightning_response.error_code.PLANNING_FAILED
                self.lightning_response_ready_event.set()
        self.done_lock.release()

    def _pfs_done_cb(self, state, result):
        self.done_lock.acquire()
        self.pfs_returned = True
        if result.status.status == result.status.SUCCESS:
            if not self.rr_returned or self.lightning_response is None:
                rospy.loginfo("Lightning: Got a path from PFS")
                
                self._send_stop_rr_planning()

                pfsPath = [p.values for p in result.path]
                shortcut = self.shortcut_path_wrapper.shortcut_path(pfsPath, self.current_group_name)
                
                self.lightning_response = self._create_get_motion_plan_response(shortcut)
                self.lightning_response_ready_event.set()
                
                self.done_lock.release()
                
                #display new path in rviz
                if self.draw_points:
                    self.draw_points_wrapper.draw_points(shortcut, self.current_group_name, "final", DrawPointsWrapper.ANGLES, DrawPointsWrapper.GREEN, 0.1)

                if self.store_paths:
                    self._store_path(shortcut, [])
                return
        else:
            rospy.loginfo("Lightning: Call to PFS did not return a path")
            if not self.use_rr or (self.rr_returned and self.lightning_response is None):
                self.lightning_response = GetMotionPlanResponse()
                self.lightning_response.error_code.val = self.lightning_response.error_code.PLANNING_FAILED
                self.lightning_response_ready_event.set()
        self.done_lock.release()

    def _create_get_motion_plan_response(self, path):
        response = GetMotionPlanResponse()
        response.error_code.val = response.error_code.SUCCESS
        response.trajectory.joint_trajectory.points = []
        for pt in path:
            jtp = JointTrajectoryPoint()
            jtp.positions = pt
            response.trajectory.joint_trajectory.points.append(jtp)
        response.trajectory.joint_trajectory.joint_names = self.current_joint_names
        return response

    def _store_path(self, final_path, retrieved_path):
        store_request = ManagePathLibraryRequest()
        store_request.joint_names = self.current_joint_names
        store_request.robot_name = self.robot_name
        store_request.action = store_request.ACTION_STORE
        for point in final_path:
            jtp = JointTrajectoryPoint()
            jtp.positions = point
            store_request.path_to_store.append(jtp)
        for point in retrieved_path:
            jtp = JointTrajectoryPoint()
            jtp.positions = point
            store_request.retrieved_path.append(jtp)
        self.manage_library_client(store_request)
        
    def _send_stop_pfs_planning(self):
        self.stop_pfs_publisher.publish(self._create_stop_planning_message())

    def _send_stop_rr_planning(self):
        self.stop_rr_publisher.publish(self._create_stop_planning_message())

    def _create_stop_planning_message(self):
        stop_message = StopPlanning()
        stop_message.planner_id = PLANNER_CONFIG_NAME
        stop_message.group_name = self.current_group_name
        return stop_message

if __name__ == "__main__":
    try:
        rospy.init_node("lightning");
        light = Lightning();
        rospy.loginfo("Lightning: ready")
        rospy.spin();
    except rospy.ROSInterruptException:
        pass;

