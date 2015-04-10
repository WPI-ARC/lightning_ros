#!/usr/bin/env python

import sys
import rospy
from moveit_msgs.srv import GetMotionPlan, GetMotionPlanRequest
from moveit_msgs.msg import JointConstraint, Constraints
from std_msgs.msg import Float64

def quick_motion_test_client():
  rospy.wait_for_service('/lightning/lightning_get_path')
  print "Working on it..."
  req = GetMotionPlanRequest()
  req.motion_plan_request.group_name = "torso"
  req.motion_plan_request.start_state.joint_state.name = ["torso_lift_joint"]
  req.motion_plan_request.start_state.joint_state.position = [float(0.2)]
  tempConstraint = JointConstraint()
  tempConstraint.joint_name = req.motion_plan_request.start_state.joint_state.name[0]
  tempConstraint.position = float(0.15)
  fullConstraint = Constraints()
  fullConstraint.joint_constraints = [tempConstraint]
  req.motion_plan_request.goal_constraints = [fullConstraint]
  req.motion_plan_request.allowed_planning_time = 50.0
  try:
    client_func = rospy.ServiceProxy('/lightning/lightning_get_path', GetMotionPlan)
    res = client_func(req)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

  print "Done!!"
  print res

if __name__=="__main__":
  quick_motion_test_client()
