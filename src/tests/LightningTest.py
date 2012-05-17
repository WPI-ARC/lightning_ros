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
        self.box_adder = BoxAdder()
        self.lightning_client = rospy.ServiceProxy(LIGHTNING_NAME, GetMotionPlan)

    #if the tester is stopped while waiting for planning, then gazebo physics may need to be unpaused
    #to unpause physics, run "rosservice call /gazebo/unpause_physics" in the command line
    def _do_single_test(self, s, g, no_movement, group_name, joint_names, controller_name, planning_time):
        subprocess.check_call("rosservice call /gazebo/pause_physics", shell=True)
        request = self._create_get_motion_plan_request(s, g, group_name, joint_names, planning_time)
        rospy.wait_for_service(LIGHTNING_NAME)
        try:
            response = self.lightning_client(request)
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
            rospy.loginfo("Lightning tester: got path with start %s, goal %s" % (path[0], path[-1]))
            if not no_movement:
                self._step_path_automatic(path, controller_name)
            return True

    def _create_get_motion_plan_request(self, start_point, goal_point, group_name, joint_names, planning_time):
        req = GetMotionPlanRequest()
        req.motion_plan_request.group_name = group_name

        req.motion_plan_request.start_state.joint_state.header.stamp = rospy.get_rostime()
        req.motion_plan_request.start_state.joint_state.name = joint_names
        req.motion_plan_request.start_state.joint_state.position = start_point

        req.motion_plan_request.goal_constraints.joint_constraints = []
        for i in xrange(len(joint_names)):
            tempConstraint = JointConstraint()
            tempConstraint.joint_name = joint_names[i]
            tempConstraint.position = goal_point[i]
            req.motion_plan_request.goal_constraints.joint_constraints.append(tempConstraint)

        req.motion_plan_request.allowed_planning_time = rospy.Duration(planning_time)
        return req

    #make sure the joint controller is turned off before calling this
    def move_to_joint_configs(self, controller_name, angles):
        set_model_config_client = rospy.ServiceProxy(SET_MODEL_CONFIGURATION, SetModelConfiguration)
        req = SetModelConfigurationRequest()
        req.model_name = "pr2"
        req.urdf_param_name = "robot_description"
        req.joint_names = rospy.get_param("/%s/joints" % (controller_name))
        req.joint_positions = angles
        rospy.wait_for_service(SET_MODEL_CONFIGURATION)
        res = set_model_config_client(req)

    def switch_off_controller(self, name):
        switch_off_client = rospy.ServiceProxy(SWITCH_CONTROLLER, SwitchController)
        req = SwitchControllerRequest()
        req.stop_controllers.append(name)
        req.strictness = req.BEST_EFFORT
        rospy.wait_for_service(SWITCH_CONTROLLER)
        res = switch_off_client(req)

    def wait_for_lightning(self):
        rospy.wait_for_service(LIGHTNING_NAME)

    def _do_ik(self, px, py, pz, arm):
        IK_INFO_NAME = "pr2_%s_kinematics/get_ik_solver_info" % (arm)
        IK_NAME = "pr2_%s_kinematics/get_constraint_aware_ik" % (arm)

        ik_solver_info_service_proxy = rospy.ServiceProxy(IK_INFO_NAME, GetKinematicSolverInfo)
        ik_info_req = GetKinematicSolverInfoRequest()
        rospy.wait_for_service(IK_INFO_NAME)
        ik_info_res = ik_solver_info_service_proxy(ik_info_req)
        
        ik_solver_service_proxy = rospy.ServiceProxy(IK_NAME, GetConstraintAwarePositionIK)
        ik_solve_req = GetConstraintAwarePositionIKRequest()
        ik_solve_req.timeout = rospy.Duration(5.0)
        ik_solve_req.ik_request.ik_link_name = "%s_wrist_roll_link" % (arm[0])
        ik_solve_req.ik_request.pose_stamped.header.frame_id = "odom_combined"
        ik_solve_req.ik_request.pose_stamped.pose.position.x = px
        ik_solve_req.ik_request.pose_stamped.pose.position.y = py
        ik_solve_req.ik_request.pose_stamped.pose.position.z = pz
        ik_solve_req.ik_request.pose_stamped.pose.orientation.x = 0.0;
        ik_solve_req.ik_request.pose_stamped.pose.orientation.y = 0.0;
        ik_solve_req.ik_request.pose_stamped.pose.orientation.z = 0.0;
        ik_solve_req.ik_request.pose_stamped.pose.orientation.w = 1.0;
        ik_solve_req.ik_request.ik_seed_state.joint_state.name = ik_info_res.kinematic_solver_info.joint_names;
        for i in xrange(len(ik_info_res.kinematic_solver_info.joint_names)):
            ik_solve_req.ik_request.ik_seed_state.joint_state.position.append((ik_info_res.kinematic_solver_info.limits[i].min_position + ik_info_res.kinematic_solver_info.limits[i].max_position)/2.0)
        
        rospy.wait_for_service(IK_NAME)
        ik_solve_res = ik_solver_service_proxy(ik_solve_req)
        if ik_solve_res.error_code.val == ik_solve_res.error_code.SUCCESS:
            return ik_solve_res.solution.joint_state.position
        else:
            rospy.loginfo("Lightning tester: inverse kinematics failed %i", ik_solve_res.error_code.val)
            return []

    def _reset_box_scene(self):
        self.box_adder.reset_box_scene()
        return 0

    def _sample_table_scene(self, n, group_name):
        if group_name == "right_arm":
            x_range, y_range, z = (0.4, 0.8), (-0.8, 0.2), TABLE_LEVEL
        elif group_name == "left_arm":
            x_range, y_range, z = (0.4, 0.8), (-0.2, 0.8), TABLE_LEVEL
        else:
            rospy.loginfo("Lightning test: sample table scene: invalid group name")
            return 0
        box_positions = []
        counter = 0
        while len(box_positions) < n:
            new_position = (x_range[0]+random()*(x_range[1]-x_range[0]), y_range[0]+random()*(y_range[1]-y_range[0]), z)
            if not self._in_collision(new_position, box_positions):
                box_positions.append(new_position)
            counter += 1
            if counter > 200: #if tried too many times to get a set of boxes, then reset and try again
                counter = 0
                box_positions = []
                rospy.loginfo("Lightning tester: trying to sample boxes again")
        self.box_adder.set_table_scene(box_positions)
        return 0

    #check if box is in collision with boxes
    def _in_collision(self, box, boxes):
        size = SMALL_BOX_SIZE
        for b in boxes:
            if abs(b[0]-box[0]) < size or abs(b[1]-box[1]) < size:
                return True
        return False

    def _step_path_manual(self, path, controller_name):
        index = 0
        move_num = 1
        while index < len(path):
            move = raw_input("Point %i of %i (enter nothing to repeat last step or type f to finish): " % (index, len(path)))
            if move != "":
                if move == 'f':
                    move_num = len(path)
                else:
                    try:
                        move_num = int(move)
                    except ValueError:
                        rospy.loginfo("Invalid input")
                        continue
            if move_num > 0:
                for i in xrange(move_num):
                    if index < len(path):
                        self.move_to_joint_configs(controller_name, path[index])
                        index += 1
                        rospy.sleep(0.005)
                    else:
                        break
            elif move_num < 0:
                for i in xrange(0, move_num, -1):
                    if index > 0:
                        index -= 1
                        self.move_to_joint_configs(controller_name, path[index])
                        rospy.sleep(0.005)
                    else:
                        break

    def _step_path_automatic(self, path, controller_name):
        for point in path:
            self.move_to_joint_configs(controller_name, point)
            rospy.sleep(0.005)

    def _sample_box_goal(self, arm, y_offset=0):
        if arm == "right_arm":
            x_range, y_range, z_range = (0.55, 0.7), (-0.2, 0.2), (0.8, 1.1)
        elif arm == "left_arm":
            x_range, y_range, z_range = (0.55, 0.7), (-0.2, 0.2), (0.8, 1.1)
        else:
            rospy.loginfo("Lightning test: sample box goal: invalid group name: %s" % (group_name))
            return []
        rand_x = x_range[0]+(x_range[1]-x_range[0])*random()
        rand_y = y_offset+y_range[0]+(y_range[1]-y_range[0])*random()
        rand_z = z_range[0]+(z_range[1]-z_range[0])*random()
        return list(self._do_ik(rand_x, rand_y, rand_z, arm))

    def _sample_table_goal(self, arm, y_offset=0):
        if arm == "right_arm":
            x_range, y_range, z = (0.4, 0.8), (-0.6, -0.1), TABLE_LEVEL+0.02
        elif arm == "left_arm":
            x_range, y_range, z = (0.4, 0.8), (0.1, 0.6), TABLE_LEVEL+0.02
        else:
            rospy.loginfo("Lightning test: sample table goal: invalid group name: %s" % (group_name))
            return []
        rand_x = x_range[0]+(x_range[1]-x_range[0])*random()
        rand_y = y_range[0]+(y_range[1]-y_range[0])*random()
        return list(self._do_ik(rand_x, rand_y+y_offset, z, arm))

    #make sure to turn off the arm controller before calling runIterations
    def _run_iterations_common(self, s, n, sample_box_func, sample_goal_func, no_movement, waiting_time, group_name, joint_names, controller_name, planning_time):
        rospy.loginfo("Lightning tester: number of test iterations = %i" % (n))
        self.move_to_joint_configs(controller_name, s)
        max_counter = 100
        i = 0
        while i < n:
            rospy.loginfo("On iteration %i" % (i))
            rospy.loginfo("Trying new setup")
            counter = 0
            if not no_movement:
                self.move_to_joint_configs(controller_name, s)
            y_offset = sample_box_func()
            goal = sample_goal_func(group_name, y_offset)
            while len(goal) == 0 and counter < max_counter:
                rospy.loginfo("got goal")
                goal = sample_goal_func(group_name, y_offset)
                counter += 1
            if counter < max_counter:
                rospy.loginfo("Lightning tester: goal = %s" % (str(goal)))
                found_path = self._do_single_test(s, goal, no_movement, group_name, joint_names, controller_name, planning_time)
                if found_path:
                    i += 1
                rospy.loginfo("Lightning tester: waiting...")
                time.sleep(waiting_time)
                rospy.loginfo("Lightning tester: done waiting")

    def run_iterations_box(self, s, n, no_movement, group_name, joint_names, controller_name, planning_time=60.0, waiting_time=5.0):
        self._run_iterations_common(s, n, self._reset_box_scene, self._sample_box_goal, no_movement, waiting_time, group_name, joint_names, controller_name, planning_time)

    def run_iterations_table(self, s, n, no_movement, group_name, joint_names, controller_name, planning_time=60.0, waiting_time=5.0, numBoxes=4):
        sample_box_func = (lambda: self._sample_table_scene(numBoxes, group_name))
        self._run_iterations_common(s, n, sample_box_func, self._sample_table_goal, no_movement, waiting_time, group_name, joint_names, controller_name, planning_time)

if __name__ == "__main__":
    try:
        rospy.init_node("run_test")
        rospy.loginfo("Lightning test: starting")
        tester = LightningTester()
        tester.wait_for_lightning()

        right_start = [-1.5777, 1.2081, -0.0126, -1.2829, -1.5667, -1.5655, 1.64557]
        tester.switch_off_controller(RIGHT_ARM_JOINT_CONTROLLER)
        tester.move_to_joint_configs(RIGHT_ARM_JOINT_CONTROLLER, right_start)

        left_start = [1.5777, 1.2081, -0.0126, -1.2829, 1.5667, -1.5655, 1.64557]
        tester.switch_off_controller(LEFT_ARM_JOINT_CONTROLLER)
        tester.move_to_joint_configs(LEFT_ARM_JOINT_CONTROLLER, left_start)
        
        if len(sys.argv) >= 4 and sys.argv[1].find("table") == 0 and sys.argv[2] == "right":
            tester.run_iterations_table(right_start, int(sys.argv[3]), False, "right_arm", RIGHT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_CONTROLLER, planning_time=20.0, waiting_time=1.0)
        elif len(sys.argv) >= 4 and sys.argv[1].find("box") == 0 and sys.argv[2] == "right":
            tester.run_iterations_box(right_start, int(sys.argv[3]), False, "right_arm", RIGHT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_CONTROLLER, planning_time=20.0, waiting_time=1.0)
        elif len(sys.argv) >= 4 and sys.argv[1].find("table") == 0 and sys.argv[2] == "left":
            tester.run_iterations_table(left_start, int(sys.argv[3]), False, "left_arm", LEFT_ARM_JOINT_NAMES, LEFT_ARM_JOINT_CONTROLLER, planning_time=20.0, waiting_time=1.0)
        elif len(sys.argv) >= 4 and sys.argv[1].find("box") == 0 and sys.argv[2] == "left":
            tester.run_iterations_box(left_start, int(sys.argv[3]), False, "left_arm", LEFT_ARM_JOINT_NAMES, LEFT_ARM_JOINT_CONTROLLER, planning_time=20.0, waiting_time=1.0)
        elif len(sys.argv) >= 3 and sys.argv[1].find("test") == 0:
            tester.run_iterations_box(right_start, int(sys.argv[2]), True, "right_arm", RIGHT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_CONTROLLER, planning_time=20.0, waiting_time=1.0)
        else:
            rospy.loginfo("Lightning tester: nothing to do")
    except rospy.ROSInterruptException:
        pass;
