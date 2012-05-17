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
        self.RRClient = actionlib.SimpleActionClient(RR_NODE_NAME, RRAction)
        self.PFSClient = actionlib.SimpleActionClient(PFS_NODE_NAME, PFSAction)
        self.manageLibraryClient = rospy.ServiceProxy(MANAGE_LIBRARY, ManagePathLibrary)
        self.storePaths = rospy.get_param("~store_paths")
        self.useRR = rospy.get_param("~use_RR")
        self.usePFS = rospy.get_param("~use_PFS")
        if not self.useRR and not self.usePFS:
            rospy.logerr("Lightning: at least one of use_RR and use_PFS need to be true")
        self.shortcutPathWrapper = ShortcutPathWrapper()
        self.lightningResponse = None
        self.lightningService = rospy.Service(LIGHTNING_SERVICE, GetMotionPlan, self.run)
        self.currentJointNames = []
        self.currentGroupName = ""
        self.robotName = rospy.get_param("robot_name")
        #self.lightningServer = actionlib.SimpleActionServer(LIGHTNING_SERVICE, LightningAction, execute_cb=self.run)
        self.stopRRPublisher = rospy.Publisher(STOP_RR_NAME, StopPlanning)
        self.stopPFSPublisher = rospy.Publisher(STOP_PFS_NAME, StopPlanning)
        self.RRReturned, self.PFSReturned = False, False
        self.doneLock = threading.Lock()
        self.lightningResponseReadyLock = threading.Lock()
        self.lightningResponseReadyEvent = threading.Event()

        #if drawPoints is True, then display points in rviz
        self.drawPoints = rospy.get_param("draw_points")
        if self.drawPoints:
            self.drawPointsWrapper = DrawPointsWrapper()

    def _lightningTimeout(self, time):
        self.lightningResponseReadyEvent.wait(time)
        if self.lightningResponse is None: #timeout
            if self.useRR:
                self._sendStopRRPlanning(self.currentGroupName)
            if self.usePFS:
                self._sendStopPFSPlanning(self.currentGroupName)

    def run(self, request):
        #make sure the request is valid
        startAndGoal = self._isValidMotionPlanRequest(request)
        if startAndGoal is None:
            response = GetMotionPlanResponse()
            response.error_code.val = response.error_code.PLANNING_FAILED
            return response
        s, g = startAndGoal

        self.RRReturned, self.PFSReturned = False, False
        self.lightningResponse = None
        self.lightningResponseReadyEvent.clear()
        self.currentJointNames = request.motion_plan_request.start_state.joint_state.name
        self.currentGroupName = request.motion_plan_request.group_name

        if self.drawPoints:
            self.drawPointsWrapper.clearPoints()

        #start a timer that stops planners if they take too long
        self.timer = threading.Thread(target=self._lightningTimeout, args=(request.motion_plan_request.allowed_planning_time.to_sec()))

        if self.useRR:
            rrClientGoal = RRGoal()
            rrClientGoal.start = s
            rrClientGoal.goal = g
            rrClientGoal.joint_names = self.currentJointNames
            rrClientGoal.group_name = self.currentGroupName
            rrClientGoal.allowed_planning_time = request.allowed_planning_time
            self.RRClient.wait_for_server()
            rospy.loginfo("Lightning: Sending goal to RR")
            self.RRClient.send_goal(rrClientGoal, done_cb=self.rrDoneCb)

        if self.usePFS:
            pfsClientGoal = PFSGoal()
            pfsClientGoal.start = s
            pfsClientGoal.goal = g
            pfsClientGoal.joint_names = self.currentJointNames
            pfsClientGoal.group_name = self.currentGroupName
            pfsClientGoal.allowed_planning_time = request.allowed_planning_time
            self.PFSClient.wait_for_server()
            rospy.loginfo("Lightning: Sending goal to PFS")
            self.PFSClient.send_goal(pfsClientGoal, done_cb=self.pfsDoneCb)
        
        self.lightningResponseReadyEvent.wait()
        if self.lightningResponse.error_code.val != self.lightningResponse.error_code.SUCCESS:
            rospy.loginfo("Lightning: did not find a path")
        else:
            rospy.loginfo("Lightning: Lightning is responding with a path")
        
        #self.lightningServer.set_succeeded(self.lightningResponse)
        return self.lightningResponse

    def _printError(self, msg):
        rospy.logerr("***ERROR*** %s ***ERROR***" % (msg))

    def _isValidMotionPlanRequest(self, request):
        if request.motion_plan_request.allowed_planning_time.to_sec() <= 0:
            self._printError("Lightning: requires allowed_planning_time to be greater than 0")
            return None
        
        if len(request.motion_plan_request.goal_constraints.position_constraints) > 0:
            self._printError("Lightning: does not handle position constraints")
            return None

        s = list(request.motion_plan_request.start_state.joint_state.position)
        g = []
        for jc in request.motion_plan_request.goal_constraints.joint_constraints:
            if jc.tolerance_above != 0 or jc.tolerance_below != 0:
                self._printError("Lightning: does not handle tolerances")
                return None
            else:
                g.append(jc.position)

        if len(s) == 0:
            self._printError("Lightning: did not receive a start state")
            return None

        if len(g) == 0:
            self._printError("Lightning: did not receive a goal state")
            return None
        return s, g

    def rrDoneCb(self, state, result):
        self.doneLock.acquire()
        self.RRReturned = True
        #may need to check return state here
        if result.status.status == result.status.SUCCESS:
            if not self.PFSReturned or self.lightningResponse is None:
                rospy.loginfo("Lightning: Got a path from RR")

                self._sendStopPFSPlanning(self.currentGroupName)

                rrPath = [p.values for p in result.repaired_path]
                shortcut = self.shortcutPathWrapper.shortcutPath(rrPath, self.currentGroupName)

                self.lightningResponse = self._createGetMotionPlanResponse(shortcut)
                self.lightningResponseReadyEvent.set()
                
                self.doneLock.release()
                
                #display new path in rviz
                if self.drawPoints:
                    self.drawPointsWrapper.drawPoints(shortcut, self.currentGroupName, "final", DrawPointsWrapper.ANGLES, DrawPointsWrapper.GREEN, 0.1)

                if self.storePaths:
                    self.storePath(shortcut, rrPath, self.currentJointNames)
                return
        else:
            rospy.loginfo("Lightning: Call to RR did not return a path")
            if not self.usePFS or (self.PFSReturned and self.lightningResponse is None):
                self.lightningResponse = GetMotionPlanResponse()
                self.lightningResponse.error_code.val = self.lightningResponse.error_code.PLANNING_FAILED
                self.lightningResponseReadyEvent.set()
        self.doneLock.release()

    def pfsDoneCb(self, state, result):
        self.doneLock.acquire()
        self.PFSReturned = True
        #may need to check return state here
        if result.status.status == result.status.SUCCESS:
            if not self.RRReturned or self.lightningResponse is None:
                rospy.loginfo("Lightning: Got a path from PFS")
                
                self._sendStopRRPlanning(self.currentGroupName)

                pfsPath = [p.values for p in result.path]
                shortcut = self.shortcutPathWrapper.shortcutPath(pfsPath, self.currentGroupName)
                
                self.lightningResponse = self._createGetMotionPlanResponse(shortcut)
                self.lightningResponseReadyEvent.set()
                
                self.doneLock.release()
                
                #display new path in rviz
                if self.drawPoints:
                    self.drawPointsWrapper.drawPoints(shortcut, self.currentGroupName, "final", DrawPointsWrapper.ANGLES, DrawPointsWrapper.GREEN, 0.1)

                if self.storePaths:
                    self.storePath(shortcut, [], self.currentJointNames)
                return
        else:
            rospy.loginfo("Lightning: Call to PFS did not return a path")
            if not self.useRR or (self.RRReturned and self.lightningResponse is None):
                self.lightningResponse = GetMotionPlanResponse()
                self.lightningResponse.error_code.val = self.lightningResponse.error_code.PLANNING_FAILED
                self.lightningResponseReadyEvent.set()
        self.doneLock.release()

    def _createGetMotionPlanResponse(self, path):
        response = GetMotionPlanResponse()
        response.error_code.val = response.error_code.SUCCESS
        response.trajectory.joint_trajectory.points = []
        for pt in path:
            jtp = JointTrajectoryPoint()
            jtp.positions = pt
            response.trajectory.joint_trajectory.points.append(jtp)
        response.trajectory.joint_trajectory.joint_names = self.currentJointNames
        return response

    def storePath(self, finalPath, retrievedPath, jointNames):
        storeRequest = ManagePathLibraryRequest()
        storeRequest.joint_names = jointNames
        storeRequest.robot_name = self.robotName
        storeRequest.action = storeRequest.ACTION_STORE
        for point in finalPath:
            jtp = JointTrajectoryPoint()
            jtp.positions = point
            storeRequest.path_to_store.append(jtp)
        for point in retrievedPath:
            jtp = JointTrajectoryPoint()
            jtp.positions = point
            storeRequest.retrieved_path.append(jtp)
        self.manageLibraryClient(storeRequest)
        
    def _sendStopPFSPlanning(self):
        self.stopPFSPublisher.publish(self._createStopPlanningMessage())

    def _sendStopRRPlanning(self):
        self.stopRRPublisher.publish(self._createStopPlanningMessage())

    def _createStopPlanningMessage(self):
        stopMessage = StopPlanning()
        stopMessage.planner_id = PLANNER_CONFIG_NAME
        stopMessage.group_name = self.currentGroupName
        return stopMessage

if __name__ == "__main__":
    try:
        rospy.init_node("lightning");
        light = Lightning();
        rospy.loginfo("Lightning: ready")
        rospy.spin();
    except rospy.ROSInterruptException:
        pass;

