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

"""
This node advertises an action which is used by the main lightning node
(see run_lightning.py) to run the Retrieve and Repair portion of LightningROS.
This node relies on a planner_stoppable type node to repair the paths, the
PathTools library to retrieve paths from the library (this is not a separate
node; just a python library that it calls), and the PathTools python library
which calls the collision_checker service and advertises a topic for displaying
stuff in RViz.
"""

import roslib
import rospy
import actionlib
import threading

from tools.PathTools import PlanTrajectoryWrapper, InvalidSectionWrapper, DrawPointsWrapper
from pathlib.PathLibrary import *
from lightning.msg import Float64Array, RRAction, RRResult
from lightning.msg import StopPlanning, RRStats
from lightning.srv import ManagePathLibrary, ManagePathLibraryResponse

import sys
import pickle
import time

# Name of this node.
RR_NODE_NAME = "rr_node"
# Name to use for stopping the repair planner. Published from this node.
STOP_PLANNER_NAME = "stop_rr_planning"
# Topic to subscribe to for stopping the whole node in the middle of processing.
STOP_RR_NAME = "stop_all_rr"
# Name of library managing service run from this node.
MANAGE_LIBRARY = "manage_path_library"
STATE_RETRIEVE, STATE_REPAIR, STATE_RETURN_PATH, STATE_FINISHED, STATE_FINISHED = (0, 1, 2, 3, 4)

class RRNode:
    def __init__(self):
        # Retrieve ROS parameters and configuration and cosntruct various objects.
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
        self.stop_rr_planner_publisher = rospy.Publisher(STOP_PLANNER_NAME, StopPlanning, queue_size=10)
        self.manage_library_service = rospy.Service(MANAGE_LIBRARY, ManagePathLibrary, self._do_manage_action)
        self.stats_pub = rospy.Publisher("rr_stats", RRStats, queue_size=10)
        self.repaired_sections_lock = threading.Lock()
        self.repaired_sections = []
        self.working_lock = threading.Lock() #to ensure that node is not doing RR and doing a library management action at the same time

        #if draw_points is True, then display points in rviz
        self.draw_points = rospy.get_param("draw_points")
        if self.draw_points:
            self.draw_points_wrapper = DrawPointsWrapper()

    def _set_repaired_section(self, index, section):
        """
          After you have done the path planning to repair a section, store
            the repaired path section.

          Args:
            index (int): the index corresponding to the section being repaired.
            section (path, list of list of float): A path to store.
        """
        self.repaired_sections_lock.acquire()
        self.repaired_sections[index] = section
        self.repaired_sections_lock.release()

    def _call_planner(self, start, goal, planning_time):
        """
          Calls a standard planner to plan between two points with an allowed
            planning time.

          Args:
            start (list of float): A joint configuration corresponding to the
              start position of the path.
            goal (list of float): The jount configuration corresponding to the
              goal position for the path.

          Returns:
            path: A list of joint configurations corresponding to the planned
              path.
        """
        ret = None
        planner_number = self.plan_trajectory_wrapper.acquire_planner()
        if not self._need_to_stop():
            ret = self.plan_trajectory_wrapper.plan_trajectory(start, goal, planner_number, self.current_joint_names, self.current_group_name, planning_time, self.planner_config_name)
        self.plan_trajectory_wrapper.release_planner(planner_number)
        return ret

    def _repair_thread(self, index, start, goal, start_index, goal_index, planning_time):
        """
          Handles repairing a portion of the path.
          All that this function really does is to plan from scratch between
            the start and goal configurations and then store the planned path
            in the appropriate places and draws either the repaired path or, if
            the repair fails, the start and goal.

          Args:
            index (int): The index to pass to _set_repaired_section(),
              corresponding to which of the invalid sections of the path we are
              repairing.
            start (list of float): The start joint configuration to use.
            goal (list of float): The goal joint configuration to use.
            start_index (int): The index in the overall path corresponding to
              start. Only used for debugging info.
            goal_index (int): The index in the overall path corresponding to
              goal. Only used for debugging info.
            planning_time (float): Maximum allowed time to spend planning, in
              seconds.
        """
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
        """
          Draws the points from the various paths involved in the planning
            in different colors in different namespaces.
          All of the arguments are lists of joint configurations, where each
            joint configuration is a list of joint angles.
          The only distinction between the different arguments being passed in
            are which color the points in question are being drawn in.
          Uses the DrawPointsWrapper to draw the points.

          Args:
            projected (list of list of float): List of points to draw as
              projected between the library path and the actual start/goal
              position. Will be drawn in blue.
            retrieved (list of list of float): The path retrieved straight
              from the path library. Will be drawn in white.
            invalid (list of list of float): List of points which were invalid.
              Will be drawn in red.
        """
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
        """
          Callback which performs the full Retrieve and Repair for the path.
        """
        self.working_lock.acquire()
        self.start_time = time.time()
        self.stats_msg = RRStats()
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

        self.stats_msg.init_time = time.time() - self.start_time

        # Go through the retrieve, repair, and return stages of the planning.
        # The while loop should only ever go through 3 iterations, one for each
        #   stage.
        while not self._need_to_stop() and repair_state != STATE_FINISHED:
            if repair_state == STATE_RETRIEVE:
                start_retrieve = time.time()
                projected, retrieved, invalid = self.path_library.retrieve_path(s, g, self.num_paths_checked, self.robot_name, self.current_group_name, self.current_joint_names)
                self.stats_msg.retrieve_time.append(time.time() - start_retrieve)
                if len(projected) == 0:
                    rospy.loginfo("RR action server: got an empty path for retrieve state")
                    repair_state = STATE_FINISHED
                else:
                    start_draw = time.time()
                    if self.draw_points:
                        self.do_retrieved_path_drawing(projected, retrieved, invalid)
                    self.stats_msg.draw_time.append(time.time() - start_draw)
                    repair_state = STATE_REPAIR
            elif repair_state == STATE_REPAIR:
                start_repair = time.time()
                repaired = self._path_repair(projected, action_goal.allowed_planning_time.to_sec(), invalid_sections=invalid)
                self.stats_msg.repair_time.append(time.time() - start_repair)
                if repaired is None:
                    rospy.loginfo("RR action server: path repair didn't finish")
                    repair_state = STATE_FINISHED
                else:
                    repair_state = STATE_RETURN_PATH
            elif repair_state == STATE_RETURN_PATH:
                start_return = time.time()
                res.status.status = res.status.SUCCESS
                res.retrieved_path = [Float64Array(p) for p in retrieved]
                res.repaired_path = [Float64Array(p) for p in repaired]
                rospy.loginfo("RR action server: returning a path")
                repair_state = STATE_FINISHED
                self.stats_msg.return_time = time.time() - start_return
        if repair_state == STATE_RETRIEVE:
            rospy.loginfo("RR action server: stopped before it retrieved a path")
        elif repair_state == STATE_REPAIR:
            rospy.loginfo("RR action server: stopped before it could repair a retrieved path")
        elif repair_state == STATE_RETURN_PATH:
            rospy.loginfo("RR action server: stopped before it could return a repaired path")
        self.rr_server.set_succeeded(res)
        self.stats_msg.total_time = time.time() - self.start_time
        self.stats_pub.publish(self.stats_msg)
        self.working_lock.release()

    def _path_repair(self, original_path, planning_time, invalid_sections=None, use_parallel_repairing=True):
        """
          Goes through each invalid section in a path and calls a planner to
            repair it, with the potential for multi-threading. Returns the
            repaired path.

          Args:
            original_path (path): The original path which needs repairing.
            planning_time (float): The maximum allowed planning time for
              each repair, in seconds.
            invalid_sections (list of pairs of indicies): The pairs of indicies
              describing the invalid sections. If None, then the invalid
              sections will be computed by this function.
            use_parallel_repairing (bool): Whether or not to use multi-threading.

          Returns:
            path: The repaired path.
        """
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
        """
          Processes a ManagePathLibraryRequest as part of the ManagePathLibrary
            service. Basically, either stores a path in the library or deletes it.
        """
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
                    store_path_result = self.path_library.store_path(new_path, request.robot_name, request.joint_names)
                else:
                    store_path_result = self.path_library.store_path(new_path, request.robot_name, request.joint_names, [p.positions for p in request.retrieved_path])
                response.result = response.SUCCESS
                response.path_stored, response.num_library_paths = store_path_result
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
