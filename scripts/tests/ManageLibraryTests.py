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

import sys

from lightning.srv import ManagePathLibrary, ManagePathLibraryRequest

MANAGE_LIBRARY = "manage_path_library"
RIGHT_ARM_JOINT_NAMES = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'];
LEFT_ARM_JOINT_NAMES = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint'];

def delete_path(robot_name, joint_names, pid):
    manage_client = rospy.ServiceProxy(MANAGE_LIBRARY, ManagePathLibrary)
    delete_request = ManagePathLibraryRequest()
    delete_request.robot_name = robot_name
    delete_request.joint_names = joint_names
    delete_request.action = delete_request.ACTION_DELETE_PATH
    delete_request.delete_id = pid
    rospy.wait_for_service(MANAGE_LIBRARY)
    response = manage_client(delete_request)
    if response.result == response.SUCCESS:
        rospy.loginfo("Manage library test: path deletion of path %i was successful" % (pid))
    else:
        rospy.loginfo("Manage library test: path deletion of path %i was not successful" % (pid))

def delete_library(robot_name, joint_names):
    manage_client = rospy.ServiceProxy(MANAGE_LIBRARY, ManagePathLibrary)
    delete_request = ManagePathLibraryRequest()
    delete_request.robot_name = robot_name
    delete_request.joint_names = joint_names
    delete_request.action = delete_request.ACTION_DELETE_LIBRARY
    rospy.wait_for_service(MANAGE_LIBRARY)
    response = manage_client(delete_request)
    if response.result == response.SUCCESS:
        rospy.loginfo("Manage library test: path deletion of library for robot %s was successful" % (robot_name))
    else:
        rospy.loginfo("Manage library test: path deletion of library for robot %s was not successful" % (robot_name))

if __name__ == "__main__":
    rospy.init_node("manage_lib_tester")
    if len(sys.argv) >= 6 and sys.argv[1:4] == ["delete", "path", "right"]:
        delete_path(sys.argv[4], RIGHT_ARM_JOINT_NAMES, int(sys.argv[5]))
    elif len(sys.argv) >= 6 and sys.argv[1:4] == ["delete", "path", "left"]:
        delete_path(sys.argv[4], LEFT_ARM_JOINT_NAMES, int(sys.argv[5]))
    elif len(sys.argv) >= 5 and sys.argv[1:4] == ["delete", "library", "right"]:
        delete_library(sys.argv[4], RIGHT_ARM_JOINT_NAMES)
    elif len(sys.argv) >= 5 and sys.argv[1:4] == ["delete", "library", "left"]:
        delete_library(sys.argv[4], LEFT_ARM_JOINT_NAMES)
    else:
        rospy.loginfo("Manage library tester: Nothing to do")
