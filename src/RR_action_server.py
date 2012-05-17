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
roslib.load_manifest("lightning")
import rospy
import actionlib
import threading

from tools.PathTools import PlanTrajectoryWrapper, InvalidSectionWrapper, DrawPointsWrapper
from pathlib.PathLibrary import *
from lightning.msg import Float64Array, RRAction, RRResult
from lightning.msg import StopPlanning
from lightning.srv import ManagePathLibrary, ManagePathLibraryResponse

import sys
import pickle

RR_NODE_NAME = "rr_node"
STOP_PLANNER_NAME = "stop_rr_planning"
STOP_RR_NAME = "stop_all_rr"
MANAGE_LIBRARY = "manage_path_library"
STATE_RETRIEVE, STATE_REPAIR, STATE_RETURN_PATH, STATE_FINISHED, STATE_FINISHED = (0, 1, 2, 3, 4)

class RRNode:
    def __init__(self):
        self.robot_name = rospy.get_param("robot_name")
        self.planner_config_name = rospy.get_param("planner_config_name")
        self.current_joint_names = []
        self.current_group_name = ""
        self.plan_trajectory_wrapper = PlanTrajectoryWrapper("rr", int(rospy.get_param("~num_rr_planners")))
        self.invalid_section_wrapper = InvalidSectionWrapper()
        self.path_library = PathLibrary(rospy.get_param("~path_library_dir"), rospy.get_param("step_size"), node_size=int(rospy.get_param("~path_library_path_node_size")), sg_node_size=int(rospy.get_param("~path_library_sg_node_size")), dtw_dist=float(rospy.get_param("~dtw_distance")))
        self.num_paths_checked = int(rospy.get_param("~num_paths_to_collision_check"))
        self.stop_lock = threading.Lock()
        self.stop = True
        self.rr_server = actionlib.SimpleActionServer(RR_NODE_NAME, RRAction, execute_cb=self._retrieve_repair, auto_start=False)
        self.rr_server.start()
        self.stop_rr_subscriber = rospy.Subscriber(STOP_RR_NAME, StopPlanning, self._stop_rr_planner)
        self.stop_rr_planner_publisher = rospy.Publisher(STOP_PLANNER_NAME, StopPlanning)
        self.manage_library_service = rospy.Service(MANAGE_LIBRARY, ManagePathLibrary, self._do_manage_action)
        self.repaired_sections_lock = threading.Lock()
        self.repaired_sections = []
        self.working_lock = threading.Lock() #to ensure that node is not doing RR and doing a library management action at the same time
        
        #if draw_points is True, then display points in rviz
        self.draw_points = rospy.get_param("draw_points")
        if self.draw_points:
            self.draw_points_wrapper = DrawPointsWrapper()

    def _set_repaired_section(self, index, section):
        self.repaired_sections_lock.acquire()
        self.repaired_sections[index] = section
        self.repaired_sections_lock.release()

    def _call_planner(self, start, goal, planning_time):
        ret = None
        planner_number = self.plan_trajectory_wrapper.acquire_planner()
        if not self._need_to_stop():
            ret = self.plan_trajectory_wrapper.plan_trajectory(start, goal, planner_number, self.current_joint_names, self.current_group_name, planning_time, self.planner_config_name)
        self.plan_trajectory_wrapper.release_planner(planner_number)
        return ret

    def _repair_thread(self, index, start, goal, start_index, goal_index, planning_time):
        repaired_path = self._call_planner(start, goal, planning_time)
        if self.draw_points:
            if repaired_path is not None and len(repaired_path) > 0:
                rospy.loginfo("RR action server: got repaired section with start = %s, goal = %s" % (repaired_path[0], repaired_path[-1]))
                self.draw_points_wrapper.draw_points(repaired_path, self.current_group_name, "repaired"+str(start_index)+"_"+str(goal_index), DrawPointsWrapper.ANGLES, DrawPointsWrapper.GREENBLUE, 1.0, 0.01)
        else:
            if self.draw_points:
                rospy.loginfo("RR action server: path repair for section (%i, %i) failed, start = %s, goal = %s" % (start_index, goal_index, start, goal))
                self.draw_points_wrapper.draw_points([start, goal], self.current_group_name, "failed_repair"+str(start_index)+"_"+str(goal_index), DrawPointsWrapper.ANGLES, DrawPointsWrapper.GREENBLUE, 1.0)
        if self._need_to_stop():
            self._set_repaired_section(index, None)
        else:
            self._set_repaired_section(index, repaired_path)

    def _need_to_stop(self):
        self.stop_lock.acquire();
        ret = self.stop;
        self.stop_lock.release();
        return ret;

    def _set_stop_value(self, val):
        self.stop_lock.acquire();
        self.stop = val;
        self.stop_lock.release();

    def do_retrieved_path_drawing(self, projected, retrieved, invalid):
        #display points in rviz
        if len(projected) > 0:
            if self.draw_points:
                self.draw_points_wrapper.draw_points(retrieved, self.current_group_name, "retrieved", DrawPointsWrapper.ANGLES, DrawPointsWrapper.WHITE, 0.1)
                projectionDisplay = projected[:projected.index(retrieved[0])]+projected[projected.index(retrieved[-1])+1:]
                self.draw_points_wrapper.draw_points(projectionDisplay, self.current_group_name, "projection", DrawPointsWrapper.ANGLES, DrawPointsWrapper.BLUE, 0.2)
                invalidDisplay = []
                for invSec in invalid:
                    invalidDisplay += projected[invSec[0]+1:invSec[-1]]
                self.draw_points_wrapper.draw_points(invalidDisplay, self.current_group_name, "invalid", DrawPointsWrapper.ANGLES, DrawPointsWrapper.RED, 0.2)

    def _retrieve_repair(self, action_goal):
        self.working_lock.acquire()
        self._set_stop_value(False)
        if self.draw_points:
            self.draw_points_wrapper.clear_points()
        rospy.loginfo("RR action server: RR got an action goal")
        s, g = action_goal.start, action_goal.goal
        res = RRResult()
        res.status.status = res.status.FAILURE
        self.current_joint_names = action_goal.joint_names
        self.current_group_name = action_goal.group_name
        projected, retrieved, invalid = [], [], []
        repair_state = STATE_RETRIEVE

        while not self._need_to_stop() and repair_state != STATE_FINISHED:
            if repair_state == STATE_RETRIEVE:
                projected, retrieved, invalid = self.path_library.retrieve_path(s, g, self.num_paths_checked, self.robot_name, self.current_group_name, self.current_joint_names)
                if len(projected) == 0:
                    rospy.loginfo("RR action server: got an empty path for retrieve state")
                    repair_state = STATE_FINISHED
                else:
                    if self.draw_points:
                        self.do_retrieved_path_drawing(projected, retrieved, invalid)
                    repair_state = STATE_REPAIR
            elif repair_state == STATE_REPAIR:
                repaired = self._path_repair(projected, action_goal.allowed_planning_time.to_sec(), invalid_sections=invalid)
                if repaired is None:
                    rospy.loginfo("RR action server: path repair failed")
                    repair_state = STATE_FINISHED
                else:
                    repair_state = STATE_RETURN_PATH
            elif repair_state == STATE_RETURN_PATH:
                res.status.status = res.status.SUCCESS
                res.retrieved_path = [Float64Array(p) for p in retrieved]
                res.repaired_path = [Float64Array(p) for p in repaired]
                rospy.loginfo("RR action server: returning a path")
                repair_state = STATE_FINISHED
        if repair_state == STATE_RETRIEVE:
            rospy.loginfo("RR action server: stopped before it retrieved a path")
        elif repair_state == STATE_REPAIR:
            rospy.loginfo("RR action server: stopped before it could repair a retrieved path")
        elif repair_state == STATE_RETURN_PATH:
            rospy.loginfo("RR action server: stopped before it could return a repaired path")
        self.rr_server.set_succeeded(res)
        self.working_lock.release()

    def _path_repair(self, original_path, planning_time, invalid_sections=None, use_parallel_repairing=True):
        zeros_tuple = tuple([0 for i in xrange(len(self.current_joint_names))])
        rospy.loginfo("RR action server: got path with %d points" % len(original_path))
        
        if invalid_sections is None:
            invalid_sections = self.invalid_section_wrapper.getInvalidSectionsForPath(original_path, self.current_group_name)
        rospy.loginfo("RR action server: invalid sections: %s" % (str(invalid_sections)))
        if len(invalid_sections) > 0:
            if invalid_sections[0][0] == -1:
                rospy.loginfo("RR action server: Start is not a valid state...nothing can be done")
                return None
            if invalid_sections[-1][1] == len(original_path):
                rospy.loginfo("RR action server: Goal is not a valid state...nothing can be done")
                return None
            
            if use_parallel_repairing:
                #multi-threaded repairing
                self.repaired_sections = [None for i in xrange(len(invalid_sections))]
                #each thread replans an invalid section
                threadList = []
                for i, sec in enumerate(invalid_sections):
                    th = threading.Thread(target=self._repair_thread, args=(i, original_path[sec[0]], original_path[sec[-1]], sec[0], sec[-1], planning_time))
                    threadList.append(th)
                    th.start()
                for th in threadList:
                    th.join()
                #once all threads return, then the repaired sections can be combined
                for item in self.repaired_sections:
                    if item is None:
                        rospy.loginfo("RR action server: RR node was stopped during repair or repair failed")
                        return None
                #replace invalid sections with replanned sections
                new_path = original_path[0:invalid_sections[0][0]]
                for i in xrange(len(invalid_sections)):
                    new_path += self.repaired_sections[i]
                    if i+1 < len(invalid_sections):
                        new_path += original_path[invalid_sections[i][1]+1:invalid_sections[i+1][0]]
                new_path += original_path[invalid_sections[-1][1]+1:]
                self.repaired_sections = [] #reset repaired_sections
            else:
                #single-threaded repairing
                rospy.loginfo("RR action server: Got invalid sections: %s" % str(invalid_sections))
                new_path = original_path[0:invalid_sections[0][0]]
                for i in xrange(len(invalid_sections)):
                    if not self._need_to_stop():
                        #start_invalid and end_invalid must correspond to valid states when passed to the planner
                        start_invalid, end_invalid = invalid_sections[i]
                        rospy.loginfo("RR action server: Requesting path to replace from %d to %d" % (start_invalid, end_invalid))
                        repairedSection = self._call_planner(original_path[start_invalid], original_path[end_invalid])
                        if repairedSection is None:
                            rospy.loginfo("RR action server: RR section repair was stopped or failed")
                            return None
                        rospy.loginfo("RR action server: Planner returned a trajectory of %d points for %d to %d" % (len(repairedSection), start_invalid, end_invalid))
                        new_path += repairedSection
                        if i+1 < len(invalid_sections):
                            new_path += original_path[end_invalid+1:invalid_sections[i+1][0]]
                    else:
                        rospy.loginfo("RR action server: RR was stopped while it was repairing the retrieved path")
                        return None
                new_path += original_path[invalid_sections[-1][1]+1:]
            rospy.loginfo("RR action server: Trajectory after replan has %d points" % len(new_path))
        else:
            new_path = original_path
            
        rospy.loginfo("RR action server: new trajectory has %i points" % (len(new_path)))
        return new_path

    def _stop_rr_planner(self, msg):
        self._set_stop_value(True)
        rospy.loginfo("RR action server: RR node got a stop message")
        self.stop_rr_planner_publisher.publish(msg)

    def _do_manage_action(self, request):
        response = ManagePathLibraryResponse()
        response.result = response.FAILURE
        if request.robot_name == "" or len(request.joint_names) == 0:
            rospy.logerr("RR action server: robot name or joint names were not provided")
            return response

        self.working_lock.acquire()
        if request.action == request.ACTION_STORE:
            rospy.loginfo("RR action server: got a path to store in path library")
            if len(request.path_to_store) > 0:
                new_path = [p.positions for p in request.path_to_store]

                if len(request.retrieved_path) == 0:
                    #PFS won so just store the path
                    self.path_library.store_path(new_path, request.robot_name, request.joint_names)
                else:
                    self.path_library.store_path(new_path, request.robot_name, request.joint_names, [p.positions for p in request.retrieved_path])
                response.result = response.SUCCESS
            else:
                response.message = "Path to store had no points"
        elif request.action == request.ACTION_DELETE_PATH:
            rospy.loginfo("RR action server: got a request to delete path %i in the path library" % (request.delete_id))
            if self.path_library.delete_path_by_id(request.delete_id, request.robot_name, request.joint_names):
                response.result = response.SUCCESS
            else:
                response.message = "No path in the library had id %i" % (request.delete_id)
        elif request.action == request.ACTION_DELETE_LIBRARY:
            rospy.loginfo("RR action server: got a request to delete library corresponding to robot %s and joints %s" % (request.robot_name, request.joint_names))
            if self.path_library.delete_library(request.robot_name, request.joint_names):
                response.result = response.SUCCESS
            else:
                response.message = "No library corresponding to robot %s and joint names %s exists"
        else:
            rospy.logerr("RR action server: manage path library request did not have a valid action set")
        self.working_lock.release()
        return response

if __name__ == "__main__":
    try:
        rospy.init_node("rr_node")
        RRNode()
        rospy.loginfo("Retrieve-repair: ready")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
