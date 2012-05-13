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

from arm_navigation_msgs.msg import CollisionObject, CollisionObjectOperation, Shape
from arm_navigation_msgs.srv import SetPlanningSceneDiff, SetPlanningSceneDiffRequest 
from geometry_msgs.msg import Pose

from random import random

SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

MOVING_BOX_SIZE = 0.10
SMALL_BOX_SIZE = 0.1
TABLE_LEVEL = 0.7

BOX_CENTER_X = 0.85
BOX_CENTER_Y = 0.0
BOX_CENTER_Z = 0.95

class BoxAdder:
    def __init__(self):
        self.setPlanningSceneDiffClient = rospy.ServiceProxy(SET_PLANNING_SCENE_DIFF_NAME, SetPlanningSceneDiff);
        self.req = SetPlanningSceneDiffRequest()

    def clearPlanningScene(self):
        self.req = SetPlanningSceneDiffRequest()
        self.req.planning_scene_diff.collision_objects = []
        self._setPlanningScene()

    def resetBoxScene(self):
        self.req = SetPlanningSceneDiffRequest()
        self._addBoxScene()
        self._setPlanningScene()

    def setTableScene(self, newPositions):
        height = SMALL_BOX_SIZE
        side = SMALL_BOX_SIZE
        self.req = SetPlanningSceneDiffRequest()
        self._addStaticTableBoxes()
        for i in xrange(len(newPositions)):
            self._addBox("box"+str(i), newPositions[i][0], newPositions[i][1], newPositions[i][2], side, side, height)
        self._setPlanningScene()

    def _addBox(self, name, px, py, pz, sl, sw, sh):
        obj = CollisionObject()
        obj.id = name
        obj.operation.operation = CollisionObjectOperation.ADD
        obj.header.frame_id = "odom_combined"

        obstacle = Shape()
        obstacle.type = Shape.BOX
        obstacle.dimensions = (sl, sw, sh)

        pos = Pose()
        pos.position.x = px 
        pos.position.y = py 
        pos.position.z = pz 
        pos.orientation.x = 0
        pos.orientation.y = 0
        pos.orientation.z = 0
        pos.orientation.w = 1

        obj.shapes.append(obstacle)
        obj.poses.append(pos)
        self.req.planning_scene_diff.collision_objects.append(obj)

    def _addBoxScene(self):
        self._addBox("box_top", BOX_CENTER_X, BOX_CENTER_Y, BOX_CENTER_Z+0.25, 0.42, 0.42, 0.05)
        self._addBox("box_bottom", BOX_CENTER_X, BOX_CENTER_Y, BOX_CENTER_Z-0.25, 0.42, 0.42, 0.05)
        self._addBox("box_left", BOX_CENTER_X, BOX_CENTER_Y+0.25, BOX_CENTER_Z, 0.42, 0.05, 0.42)
        self._addBox("box_right", BOX_CENTER_X, BOX_CENTER_Y-0.25, BOX_CENTER_Z, 0.42, 0.05, 0.42)
        self._addBox("counter", BOX_CENTER_X-0.05, BOX_CENTER_Y, BOX_CENTER_Z-0.45, 0.8, 2.5, 0.35)

    def _addStaticTableBoxes(self):
        self._addBox("counter", 0.6, 0.0, TABLE_LEVEL-0.08, 0.5, 1.8, 0.05)

    def _setPlanningScene(self):
        while rospy.get_time() == 0:
            rospy.sleep(0.1);
        for obj in self.req.planning_scene_diff.collision_objects:
            obj.header.stamp = rospy.get_rostime()
        rospy.wait_for_service(SET_PLANNING_SCENE_DIFF_NAME)
        try:
            rospy.loginfo("Box adder: Sending new planning scene to %s" % self.setPlanningSceneDiffClient.resolved_name)
            response = self.setPlanningSceneDiffClient(self.req)
            rospy.loginfo("Box adder: collision objects: %s" % (str(len(response.planning_scene.collision_objects))))
        except rospy.ServiceException, e:
            rospy.loginfo("Box adder: Service call failed: %s"%e)
