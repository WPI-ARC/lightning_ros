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

from BoxAdder import BoxAdder
from tools.PathTools import InvalidSectionWrapper
from pathlib.PathLibrary import PathLibrary
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest
from arm_navigation_msgs.srv import GetMotionPlan, GetMotionPlanRequest
from arm_navigation_msgs.msg import JointConstraint
from pr2_mechanism_msgs.srv import SwitchController, SwitchControllerRequest
from kinematics_msgs.srv import GetKinematicSolverInfo, GetKinematicSolverInfoRequest, GetConstraintAwarePositionIK, GetConstraintAwarePositionIKRequest

from random import random
import time
import os
import sys
import subprocess

LIGHTNING_NAME = "lightning_get_path";
RIGHT_ARM_JOINT_NAMES = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'];
LEFT_ARM_JOINT_NAMES = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint'];
RIGHT_ARM_JOINT_CONTROLLER = "r_arm_controller"
LEFT_ARM_JOINT_CONTROLLER = "l_arm_controller"
SET_MODEL_STATE = "/gazebo/set_model_state"
SET_MODEL_CONFIGURATION = "/gazebo/set_model_configuration"
SWITCH_CONTROLLER = "/pr2_controller_manager/switch_controller"

STEP_SIZE = 0.02

SMALL_BOX_SIZE = 0.1
TABLE_LEVEL = 0.7

class LightningTester:

    def __init__(self):
        self.boxAdder = BoxAdder()
        self.lightningClient = rospy.ServiceProxy(LIGHTNING_NAME, GetMotionPlan)

    #if the tester is stopped while waiting for planning, then gazebo physics may need to be unpaused
    #to unpause physics, run "rosservice call /gazebo/unpause_physics" in the command line
    def _doSingleTest(self, s, g, noMovement, groupName, jointNames, controllerName):
        subprocess.check_call("rosservice call /gazebo/pause_physics", shell=True)
        request = self._createGetMotionPlanRequest(s, g, groupName, jointNames)
        rospy.wait_for_service(LIGHTNING_NAME)
        #rospy.loginfo("Lightning tester: sending request for start %s, goal %s" % (s, g)) 
        try:
            response = self.lightningClient(request)
        except rospy.ServiceException, e:
            rospy.loginfo("Lightning tester: call to lightning failed")
            subprocess.check_call("rosservice call /gazebo/unpause_physics", shell=True)
            return False
        subprocess.check_call("rosservice call /gazebo/unpause_physics", shell=True)
        path = [pt.positions for pt in response.trajectory.joint_trajectory.points]
        if len(path) == 0:
            rospy.loginfo("Lightning tester: lightning did not return a path")
            return False
        else:
            rospy.loginfo("Lightning tester: got path of with start %s, goal %s" % (path[0], path[-1]))
            if not noMovement:
                self._stepPathAutomatic(path, controllerName)
            return True

    def _createGetMotionPlanRequest(self, start_point, goal_point, groupName, jointNames):
        req = GetMotionPlanRequest()
        req.motion_plan_request.group_name = groupName

        req.motion_plan_request.start_state.joint_state.header.stamp = rospy.get_rostime()
        req.motion_plan_request.start_state.joint_state.name = jointNames
        req.motion_plan_request.start_state.joint_state.position = start_point

        req.motion_plan_request.goal_constraints.joint_constraints = []
        for i in xrange(len(jointNames)):
            tempConstraint = JointConstraint()
            tempConstraint.joint_name = jointNames[i]
            tempConstraint.position = goal_point[i]
            req.motion_plan_request.goal_constraints.joint_constraints.append(tempConstraint)
        return req

    #make sure the joint controller is turned off before calling this
    def moveToJointConfigs(self, controllerName, angles):
        setModelConfigClient = rospy.ServiceProxy(SET_MODEL_CONFIGURATION, SetModelConfiguration)
        req = SetModelConfigurationRequest()
        req.model_name = "pr2"
        req.urdf_param_name = "robot_description"
        req.joint_names = rospy.get_param("/%s/joints" % (controllerName))
        req.joint_positions = angles
        rospy.wait_for_service(SET_MODEL_CONFIGURATION)
        res = setModelConfigClient(req)

    def switchOffController(self, name):
        switchOffClient = rospy.ServiceProxy(SWITCH_CONTROLLER, SwitchController)
        req = SwitchControllerRequest()
        req.stop_controllers.append(name)
        req.strictness = req.BEST_EFFORT
        rospy.wait_for_service(SWITCH_CONTROLLER)
        res = switchOffClient(req)

    def waitForLightning(self):
        rospy.wait_for_service(LIGHTNING_NAME)

    def _doIK(self, px, py, pz, arm):
        IK_INFO_NAME = "pr2_%s_kinematics/get_ik_solver_info" % (arm)
        IK_NAME = "pr2_%s_kinematics/get_constraint_aware_ik" % (arm)

        ik_solver_info_service_proxy = rospy.ServiceProxy(IK_INFO_NAME, GetKinematicSolverInfo)
        ikInfoReq = GetKinematicSolverInfoRequest()
        rospy.wait_for_service(IK_INFO_NAME)
        ikInfoRes = ik_solver_info_service_proxy(ikInfoReq)
        
        ik_solver_service_proxy = rospy.ServiceProxy(IK_NAME, GetConstraintAwarePositionIK)
        ikSolveReq = GetConstraintAwarePositionIKRequest()
        ikSolveReq.timeout = rospy.Duration(5.0)
        ikSolveReq.ik_request.ik_link_name = "%s_wrist_roll_link" % (arm[0])
        ikSolveReq.ik_request.pose_stamped.header.frame_id = "odom_combined"
        ikSolveReq.ik_request.pose_stamped.pose.position.x = px
        ikSolveReq.ik_request.pose_stamped.pose.position.y = py
        ikSolveReq.ik_request.pose_stamped.pose.position.z = pz
        ikSolveReq.ik_request.pose_stamped.pose.orientation.x = 0.0;
        ikSolveReq.ik_request.pose_stamped.pose.orientation.y = 0.0;
        ikSolveReq.ik_request.pose_stamped.pose.orientation.z = 0.0;
        ikSolveReq.ik_request.pose_stamped.pose.orientation.w = 1.0;
        ikSolveReq.ik_request.ik_seed_state.joint_state.name = ikInfoRes.kinematic_solver_info.joint_names;
        for i in xrange(len(ikInfoRes.kinematic_solver_info.joint_names)):
            ikSolveReq.ik_request.ik_seed_state.joint_state.position.append((ikInfoRes.kinematic_solver_info.limits[i].min_position + ikInfoRes.kinematic_solver_info.limits[i].max_position)/2.0)
        
        rospy.wait_for_service(IK_NAME)
        ikSolveRes = ik_solver_service_proxy(ikSolveReq)
        if ikSolveRes.error_code.val == ikSolveRes.error_code.SUCCESS:
            return ikSolveRes.solution.joint_state.position
        else:
            #rospy.loginfo("Lightning tester: inverse kinematics failed")
            return []

    def _resetBoxScene(self):
        self.boxAdder.resetBoxScene()
        return 0

    def _sampleTableScene(self, n, groupName):
        if groupName == "right_arm":
            xRange, yRange, z = (0.4, 0.8), (-0.8, 0.2), TABLE_LEVEL
        elif groupName == "left_arm":
            xRange, yRange, z = (0.4, 0.8), (-0.2, 0.8), TABLE_LEVEL
        else:
            rospy.loginfo("Lightning test: sample table scene: invalid group name")
            return 0
        boxPositions = []
        counter = 0
        while len(boxPositions) < n:
            newPosition = (xRange[0]+random()*(xRange[1]-xRange[0]), yRange[0]+random()*(yRange[1]-yRange[0]), z)
            if not self._inCollision(newPosition, boxPositions):
                boxPositions.append(newPosition)
            counter += 1
            if counter > 200: #if tried too many times to get a set of boxes, then reset and try again
                counter = 0
                boxPositions = []
                rospy.loginfo("Lightning tester: trying to sample boxes again")
        self.boxAdder.setTableScene(boxPositions)
        return 0

    def _inCollision(self, box, boxes):
        size = SMALL_BOX_SIZE
        for b in boxes:
            if abs(b[0]-box[0]) < size or abs(b[1]-box[1]) < size:
                return True
        return False

    def _stepPathManual(self, path, controllerName):
        index = 0
        moveNum = 1
        while index < len(path):
            move = raw_input("Point %i of %i (enter nothing to repeat last step or type f to finish): " % (index, len(path)))
            if move != "":
                if move == 'f':
                    moveNum = len(path)
                else:
                    try:
                        moveNum = int(move)
                    except ValueError:
                        rospy.loginfo("Invalid input")
                        continue
            if moveNum > 0:
                for i in xrange(moveNum):
                    if index < len(path):
                        self.moveToJointConfigs(controllerName, path[index])
                        index += 1
                        rospy.sleep(0.005)
                    else:
                        break
            elif moveNum < 0:
                for i in xrange(0, moveNum, -1):
                    if index > 0:
                        index -= 1
                        self.moveToJointConfigs(controllerName, path[index])
                        rospy.sleep(0.005)
                    else:
                        break

    def _stepPathAutomatic(self, path, controllerName):
        for point in path:
            self.moveToJointConfigs(controllerName, point)
            rospy.sleep(0.005)

    def _sampleBoxGoal(self, arm, yOffset=0):
        if arm == "right_arm":
            xRange, yRange, zRange = (0.55, 0.7), (-0.2, 0.2), (0.8, 1.1)
        elif arm == "left_arm":
            xRange, yRange, zRange = (0.55, 0.7), (-0.2, 0.2), (0.8, 1.1)
        else:
            rospy.loginfo("Lightning test: sample box goal: invalid group name: %s" % (groupName))
            return []
        randX = xRange[0]+(xRange[1]-xRange[0])*random()
        randY = yOffset+yRange[0]+(yRange[1]-yRange[0])*random()
        randZ = zRange[0]+(zRange[1]-zRange[0])*random()
        #rospy.loginfo("Lightning tester: sampled pose: %s" % (str((randX, randY, randZ))))
        return list(self._doIK(randX, randY, randZ, arm))

    def _sampleTableGoal(self, arm, yOffset=0):
        if arm == "right_arm":
            xRange, yRange, z = (0.4, 0.8), (-0.6, -0.1), TABLE_LEVEL+0.02
        elif arm == "left_arm":
            xRange, yRange, z = (0.4, 0.8), (0.1, 0.6), TABLE_LEVEL+0.02
        else:
            rospy.loginfo("Lightning test: sample table goal: invalid group name: %s" % (groupName))
            return []
        randX = xRange[0]+(xRange[1]-xRange[0])*random()
        randY = yRange[0]+(yRange[1]-yRange[0])*random()
        return list(self._doIK(randX, randY+yOffset, z, arm))

    #make sure to turn off the arm controller before calling runIterations
    def _runIterationsCommon(self, s, n, sampleBoxFunc, sampleGoalFunc, noMovement, waitingTime, groupName, jointNames, controllerName):
        rospy.loginfo("Lightning tester: number of test iterations = %i" % (n))
        self.moveToJointConfigs(controllerName, s)
        maxCounter = 100
        i = 0
        while i < n:
            rospy.loginfo("On iteration %i" % (i))
            rospy.loginfo("Trying new setup")
            counter = 0
            if not noMovement:
                self.moveToJointConfigs(controllerName, s)
            yOffset = sampleBoxFunc()
            goal = sampleGoalFunc(groupName, yOffset)
            while len(goal) == 0 and counter < maxCounter:
                goal = sampleGoalFunc(groupName, yOffset)
                counter += 1
            if counter < maxCounter:
                rospy.loginfo("Lightning tester: goal = %s" % (str(goal)))
                foundPath = self._doSingleTest(s, goal, noMovement, groupName, jointNames, controllerName)
                if foundPath:
                    i += 1
                rospy.loginfo("Lightning tester: waiting...")
                time.sleep(waitingTime)
                rospy.loginfo("Lightning tester: done waiting")

    def runIterationsBox(self, s, n, noMovement, groupName, jointNames, controllerName, waitingTime=5.0):
        self._runIterationsCommon(s, n, self._resetBoxScene, self._sampleBoxGoal, noMovement, waitingTime, groupName, jointNames, controllerName)

    def runIterationsTable(self, s, n, noMovement, groupName, jointNames, controllerName, waitingTime=5.0, numBoxes=4):
        sampleBoxFunc = (lambda: self._sampleTableScene(numBoxes, groupName))
        self._runIterationsCommon(s, n, sampleBoxFunc, self._sampleTableGoal, noMovement, waitingTime, groupName, jointNames, controllerName)

if __name__ == "__main__":
    try:
        rospy.init_node("run_test")
        rospy.loginfo("Lightning test: starting")
        tester = LightningTester()
        tester.waitForLightning()

        right_start = [-1.5777, 1.2081, -0.0126, -1.2829, -1.5667, -1.5655, 1.64557]
        tester.switchOffController(RIGHT_ARM_JOINT_CONTROLLER)
        tester.moveToJointConfigs(RIGHT_ARM_JOINT_CONTROLLER, right_start)

        left_start = [1.5777, 1.2081, -0.0126, -1.2829, 1.5667, -1.5655, 1.64557]
        tester.switchOffController(LEFT_ARM_JOINT_CONTROLLER)
        tester.moveToJointConfigs(LEFT_ARM_JOINT_CONTROLLER, left_start)
        
        if len(sys.argv) >= 4 and sys.argv[1].find("table") == 0 and sys.argv[2] == "right":
            tester.runIterationsTable(right_start, int(sys.argv[3]), False, "right_arm", RIGHT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_CONTROLLER, waitingTime=1.0)
        elif len(sys.argv) >= 4 and sys.argv[1].find("box") == 0 and sys.argv[2] == "right":
            tester.runIterationsBox(right_start, int(sys.argv[3]), False, "right_arm", RIGHT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_CONTROLLER, waitingTime=1.0)
        elif len(sys.argv) >= 4 and sys.argv[1].find("table") == 0 and sys.argv[2] == "left":
            tester.runIterationsTable(left_start, int(sys.argv[3]), False, "left_arm", LEFT_ARM_JOINT_NAMES, LEFT_ARM_JOINT_CONTROLLER, waitingTime=1.0)
        elif len(sys.argv) >= 4 and sys.argv[1].find("box") == 0 and sys.argv[2] == "left":
            tester.runIterationsBox(left_start, int(sys.argv[3]), False, "left_arm", LEFT_ARM_JOINT_NAMES, LEFT_ARM_JOINT_CONTROLLER, waitingTime=1.0)
        else:
            rospy.loginfo("Lightning tester: nothing to do")
    except rospy.ROSInterruptException:
        pass;
