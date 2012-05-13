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

COLLISION_CHECK_NODE = "collision_check"
RR_NODE_NAME = "rr_node"
STOP_PLANNER_NAME = "stop_rr_planning"
STOP_RR_NAME = "stop_all_rr"
RETRIEVE_PATH_NAME = "retrieve_path"
MANAGE_LIBRARY = "manage_path_library"
STATE_RETRIEVE, STATE_REPAIR, STATE_RETURN_PATH, STATE_FINISHED, STATE_FINISHED = (0, 1, 2, 3, 4)

class RRNode:
    def __init__(self):
        self.robotName = rospy.get_param("robot_name")
        self.currentJointNames = []
        self.currentGroupName = ""
        self.planTrajectoryWrapper = PlanTrajectoryWrapper("rr", int(rospy.get_param("~num_rr_planners")))
        self.invalidSectionWrapper = InvalidSectionWrapper()
        self.pathLibrary = PathLibrary(rospy.get_param("path_library_dir"), rospy.get_param("step_size"), nodeSize=int(rospy.get_param("~path_library_path_node_size")), sgNodeSize=int(rospy.get_param("~path_library_sg_node_size")), dtwDist=float(rospy.get_param("dtw_distance")))
        self.numPathsChecked = int(rospy.get_param("~num_paths_to_collision_check"))
        self.stopLock = threading.Lock()
        self.stop = True
        self.RRServer = actionlib.SimpleActionServer(RR_NODE_NAME, RRAction, execute_cb=self.retrieveRepair, auto_start=False)
        self.RRServer.start()
        self.stopRRSubscriber = rospy.Subscriber(STOP_RR_NAME, StopPlanning, self.stopRRPlanner)
        self.stopRRPlannerPublisher = rospy.Publisher(STOP_PLANNER_NAME, StopPlanning)
        self.manageLibraryServer = rospy.Service(MANAGE_LIBRARY, ManagePathLibrary, self.doManageAction)
        self.repairedSectionsLock = threading.Lock()
        self.repairedSections = []
        self.workingLock = threading.Lock() #to ensure that node is not doing RR and doing a library management action at the same time
        
        #if drawPoints is True, then display points in rviz
        self.drawPoints = rospy.get_param("draw_points")
        if self.drawPoints:
            self.drawPointsWrapper = DrawPointsWrapper()

    def _setRepairedSection(self, index, section):
        self.repairedSectionsLock.acquire()
        self.repairedSections[index] = section
        self.repairedSectionsLock.release()

    def _callPlanner(self, start, goal):
        ret = None
        plannerNumber = self.planTrajectoryWrapper.acquirePlanner()
        if not self._needToStop():
            ret = self.planTrajectoryWrapper.planTrajectory(start, goal, plannerNumber, self.currentJointNames, self.currentGroupName)
        self.planTrajectoryWrapper.releasePlanner(plannerNumber)
        return ret

    def _threadActivity(self, index, start, goal, startIndex, goalIndex):
        repairedPath = self._callPlanner(start, goal)
        if self.drawPoints:
            if repairedPath is not None and len(repairedPath) > 0:
                rospy.loginfo("RR action server: got repaired section with start = %s, goal = %s" % (repairedPath[0], repairedPath[-1]))
                self.drawPointsWrapper.drawPoints(repairedPath, self.currentGroupName, "repaired"+str(startIndex)+"_"+str(goalIndex), DrawPointsWrapper.ANGLES, DrawPointsWrapper.GREENBLUE, 1.0, 0.01)
        else:
            if self.drawPoints:
                rospy.loginfo("RR action server: path repair for section (%i, %i) failed, start = %s, goal = %s" % (startIndex, goalIndex, start, goal))
                self.drawPointsWrapper.drawPoints([start, goal], self.currentGroupName, "failed_repair"+str(startIndex)+"_"+str(goalIndex), DrawPointsWrapper.ANGLES, DrawPointsWrapper.GREENBLUE, 1.0)
        if self._needToStop():
            self._setRepairedSection(index, None)
        else:
            self._setRepairedSection(index, repairedPath)

    def _needToStop(self):
        self.stopLock.acquire();
        ret = self.stop;
        self.stopLock.release();
        return ret;

    def _setStopValue(self, val):
        self.stopLock.acquire();
        self.stop = val;
        self.stopLock.release();

    def _doRetrievedPathDrawing(self, projected, retrieved, invalid):
        #display points in rviz
        if len(projected) > 0:
            if self.drawPoints:
                self.drawPointsWrapper.drawPoints(retrieved, self.currentGroupName, "retrieved", DrawPointsWrapper.ANGLES, DrawPointsWrapper.WHITE, 0.1)
                projectionDisplay = projected[:projected.index(retrieved[0])]+projected[projected.index(retrieved[-1])+1:]
                self.drawPointsWrapper.drawPoints(projectionDisplay, self.currentGroupName, "projection", DrawPointsWrapper.ANGLES, DrawPointsWrapper.BLUE, 0.2)
                invalidDisplay = []
                for invSec in invalid:
                    invalidDisplay += projected[invSec[0]+1:invSec[-1]]
                self.drawPointsWrapper.drawPoints(invalidDisplay, self.currentGroupName, "invalid", DrawPointsWrapper.ANGLES, DrawPointsWrapper.RED, 0.2)


    def retrieveRepair(self, actionGoal):
        self.workingLock.acquire()
        self._setStopValue(False)
        if self.drawPoints:
            self.drawPointsWrapper.clearPoints()
        rospy.loginfo("RR action server: RR got an action goal")
        s, g = actionGoal.start, actionGoal.goal
        res = RRResult()
        res.status.status = res.status.FAILURE
        self.currentJointNames = actionGoal.joint_names
        self.currentGroupName = actionGoal.group_name
        projected, retrieved, invalid = [], [], []
        repairState = STATE_RETRIEVE

        while not self._needToStop() and repairState != STATE_FINISHED:
            if repairState == STATE_RETRIEVE:
                projected, retrieved, invalid = self.pathLibrary.retrievePath(s, g, self.numPathsChecked, self.robotName, self.currentGroupName, self.currentJointNames)
                if len(projected) == 0:
                    rospy.loginfo("RR action server: got an empty path for retrieve state")
                    repairState = STATE_FINISHED
                else:
                    if self.drawPoints:
                        self._doRetrievedPathDrawing(projected, retrieved, invalid)
                    repairState = STATE_REPAIR
            elif repairState == STATE_REPAIR:
                repaired = self.pathRepair(projected, invalidSections=invalid)
                if repaired is None:
                    rospy.loginfo("RR action server: path repair failed")
                    repairState = STATE_FINISHED
                else:
                    repairState = STATE_RETURN_PATH
            elif repairState == STATE_RETURN_PATH:
                res.status.status = res.status.SUCCESS
                res.retrieved_path = [Float64Array(p) for p in retrieved]
                res.repaired_path = [Float64Array(p) for p in repaired]
                rospy.loginfo("RR action server: returning a path")
                repairState = STATE_FINISHED
        if repairState == STATE_RETRIEVE:
            rospy.loginfo("RR action server: stopped before it retrieved a path")
        elif repairState == STATE_REPAIR:
            rospy.loginfo("RR action server: stopped before it could repair a retrieved path")
        elif repairState == STATE_RETURN_PATH:
            rospy.loginfo("RR action server: stopped before it could return a repaired path")
        self.RRServer.set_succeeded(res)
        self.workingLock.release()

    def pathRepair(self, origPath, invalidSections=None, useParallelReparing=True):
        zeros_tuple = tuple([0 for i in xrange(len(self.currentJointNames))])
        rospy.loginfo("RR action server: got path with %d points" % len(origPath))
        
        if invalidSections is None:
            invalidSections = self.invalidSectionWrapper.getInvalidSectionsForPath(origPath, self.currentGroupName)
        rospy.loginfo("RR action server: invalid sections: %s" % (str(invalidSections)))
        if len(invalidSections) > 0:
            if invalidSections[0][0] == -1:
                rospy.loginfo("RR action server: Start is not a valid state...nothing can be done")
                return None
            if invalidSections[-1][1] == len(origPath):
                rospy.loginfo("RR action server: Goal is not a valid state...nothing can be done")
                return None
            
            if useParallelReparing:
                #multi-threaded repairing
                self.repairedSections = [None for i in xrange(len(invalidSections))]
                #each thread replans an invalid section
                threadList = []
                for i, sec in enumerate(invalidSections):
                    th = threading.Thread(target=self._threadActivity, args=(i, origPath[sec[0]], origPath[sec[-1]], sec[0], sec[-1]))
                    threadList.append(th)
                    th.start()
                for th in threadList:
                    th.join()
                #once all threads return, then the repaired sections can be combined
                for item in self.repairedSections:
                    if item is None:
                        rospy.loginfo("RR action server: RR node was stopped during repair or repair failed")
                        return None
                #replace invalid sections with replanned sections
                newPath = origPath[0:invalidSections[0][0]]
                for i in xrange(len(invalidSections)):
                    newPath += self.repairedSections[i]
                    if i+1 < len(invalidSections):
                        newPath += origPath[invalidSections[i][1]+1:invalidSections[i+1][0]]
                newPath += origPath[invalidSections[-1][1]+1:]
                self.repairedSections = [] #reset repairedSections
            else:
                #single-threaded repairing
                rospy.loginfo("RR action server: Got invalid sections: %s" % str(invalidSections))
                newPath = origPath[0:invalidSections[0][0]]
                for i in xrange(len(invalidSections)):
                    if not self._needToStop():
                        #startInvalid and endInvalid must correspond to valid states when passed to the planner
                        startInvalid, endInvalid = invalidSections[i]
                        rospy.loginfo("RR action server: Requesting path to replace from %d to %d" % (startInvalid, endInvalid))
                        repairedSection = self._callPlanner(origPath[startInvalid], origPath[endInvalid])
                        if repairedSection is None:
                            rospy.loginfo("RR action server: RR section repair was stopped or failed")
                            return None
                        rospy.loginfo("RR action server: Planner returned a trajectory of %d points for %d to %d" % (len(repairedSection), startInvalid, endInvalid))
                        newPath += repairedSection
                        if i+1 < len(invalidSections):
                            newPath += origPath[endInvalid+1:invalidSections[i+1][0]]
                    else:
                        rospy.loginfo("RR action server: RR was stopped while it was repairing the retrieved path")
                        return None
                newPath += origPath[invalidSections[-1][1]+1:]
            rospy.loginfo("RR action server: Trajectory after replan has %d points" % len(newPath))
        else:
            newPath = origPath
            
        rospy.loginfo("RR action server: new trajectory has %i points" % (len(newPath)))
        return newPath

    def stopRRPlanner(self, msg):
        self._setStopValue(True)
        rospy.loginfo("RR action server: RR node got a stop message")
        self.stopRRPlannerPublisher.publish(msg)

    def doManageAction(self, request):
        self.workingLock.acquire()
        response = ManagePathLibraryResponse()
        response.result = response.FAILURE
        
        if request.action == request.ACTION_STORE:
            rospy.loginfo("RR action server: got a path to store in path library")
            if len(request.path_to_store) > 0:
                newPath = [p.positions for p in request.path_to_store]

                if len(request.retrieved_path) == 0:
                    #PFS won so just store the path
                    self.pathLibrary.storePath(newPath, request.robot_name, request.joint_names)
                else:
                    self.pathLibrary.storePath(newPath, request.robot_name, request.joint_names, [p.positions for p in request.retrieved_path])
                response.result = response.SUCCESS
            else:
                response.message = "Path to store had no points"
        elif request.action == request.ACTION_DELETE_PATH:
            rospy.loginfo("RR action server: got a request to delete path %i in the path library" % (request.delete_id))
            if self.pathLibrary.deletePathById(request.delete_id, request.robot_name, request.joint_names):
                response.result = response.SUCCESS
            else:
                response.message = "No path in the library had id %i" % (request.delete_id)
        elif request.action == request.ACTION_DELETE_LIBRARY:
            rospy.loginfo("RR action server: got a request to delete library corresponding to robot %s and joints %s" % (request.robot_name, request.joint_names))
            if self.pathLibrary.deleteLibrary(request.robot_name, request.joint_names):
                response.result = response.SUCCESS
            else:
                response.message = "No library corresponding to robot %s and joint names %s exists"
        else:
            rospy.logerr("RR action server: manage path library request did not have a valid action set")
        self.workingLock.release()
        return response

if __name__ == "__main__":
    try:
        rospy.init_node("rr_node")
        RRNode()
        rospy.loginfo("Retrieve-repair: ready")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
