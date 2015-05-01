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
import rospy

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from random import random

SET_PLANNING_SCENE_DIFF_NAME = "planning_scene";

MOVING_BOX_SIZE = 0.10
SMALL_BOX_SIZE = 0.1
TABLE_LEVEL = 0.6

BOX_CENTER_X = 0.85
BOX_CENTER_Y = 0
BOX_CENTER_Z = 0.95

class BoxAdder:
    """
      This class serves to add rectangular prisms (boxes) to the planning scene
        so that they can be seen in RViz and can be taken into account in the
        collision checkers.
    """

    def __init__(self):
        self.spsd_pub = rospy.Publisher(SET_PLANNING_SCENE_DIFF_NAME, PlanningScene, queue_size=10);
        self.diff = PlanningScene()
        self.diff.is_diff = True

    def clear_planning_scene(self):
        self.diff = PlanningScene()
        self.diff.is_diff = True
        self.diff.world.collision_objects = []
        self._set_planning_scene()

    def reset_box_scene(self):
        """
          Sets up the scene for BoxTest.
        """
        self.diff = PlanningScene()
        self.diff.is_diff = True
        rospy.loginfo("BoxAdder: reset_box_scene: starting")
        self._add_box_scene()
        rospy.loginfo("BoxAdder: reset_box_scene: added box scene")
        self._set_planning_scene()
        rospy.loginfo("BoxAdder: reset_box_scene: set planning scene")

    def set_table_scene(self, new_positions):
        """
          Sets up the scene for TableTest.
        """
        height = SMALL_BOX_SIZE
        side = SMALL_BOX_SIZE
        self.diff = PlanningScene()
        self.diff.is_diff = True
        self._add_static_table_boxes()
        for i in xrange(len(new_positions)):
            self._add_box("box"+str(i), new_positions[i][0], new_positions[i][1], new_positions[i][2], side, side, height)
        self._set_planning_scene()

    def _add_box(self, name, px, py, pz, sl, sw, sh):
        """
          Performs the simple adding of a single box with given dimensions.
        """
        obj = CollisionObject()
        obj.id = name
        obj.operation = obj.ADD
        obj.header.frame_id = "odom_combined"

        obstacle = SolidPrimitive()
        obstacle.type = obstacle.BOX
        obstacle.dimensions = (sl, sw, sh)

        pos = Pose()
        pos.position.x = px
        pos.position.y = py
        pos.position.z = pz
        pos.orientation.x = 0
        pos.orientation.y = 0
        pos.orientation.z = 0
        pos.orientation.w = 1

        obj.primitives.append(obstacle)
        obj.primitive_poses.append(pos)
        self.diff.world.collision_objects.append(obj)

    def _add_box_scene(self):
        """
          Creates the boxes necessary specifically for BoxTest.
        """
        self._add_box("box_top", BOX_CENTER_X, BOX_CENTER_Y, BOX_CENTER_Z+0.25, 0.42, 0.42, 0.05)
        self._add_box("box_bottom", BOX_CENTER_X, BOX_CENTER_Y, BOX_CENTER_Z-0.25, 0.42, 0.42, 0.05)
        self._add_box("box_left", BOX_CENTER_X, BOX_CENTER_Y+0.25, BOX_CENTER_Z, 0.42, 0.05, 0.42)
        self._add_box("box_right", BOX_CENTER_X, BOX_CENTER_Y-0.25, BOX_CENTER_Z, 0.42, 0.05, 0.42)
        self._add_box("counter", BOX_CENTER_X+0.10, BOX_CENTER_Y, BOX_CENTER_Z-0.45, 0.8, 2.5, 0.35)

    def _add_static_table_boxes(self):
        """
          Creates the counter used for the TableTest.
        """
        self._add_box("counter", 0.8, 0.0, TABLE_LEVEL-0.08, 0.5, 1.8, 0.05)

    def _set_planning_scene(self):
        """
          Actually publish our new boxes.
        """
        while rospy.get_time() == 0:
            rospy.sleep(0.1);
        for obj in self.diff.world.collision_objects:
            obj.header.stamp = rospy.get_rostime()
        # We don't actually use this service, but it tells us if the move_group has started.
        rospy.wait_for_service("/get_planning_scene")
        rospy.loginfo("Box adder: Sending new planning scene")
        self.spsd_pub.publish(self.diff)
