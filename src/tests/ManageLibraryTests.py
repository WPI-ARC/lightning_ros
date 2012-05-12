#!/usr/bin/env python
import roslib
roslib.load_manifest("lightning");
import rospy

import sys

from lightning.srv import ManagePathLibrary, ManagePathLibraryRequest

MANAGE_LIBRARY = "manage_path_library"
RIGHT_ARM_JOINT_NAMES = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'];
LEFT_ARM_JOINT_NAMES = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint'];

def deletePath(robotName, jointNames, pid):
    manageClient = rospy.ServiceProxy(MANAGE_LIBRARY, ManagePathLibrary)
    deleteRequest = ManagePathLibraryRequest()
    deleteRequest.robot_name = robotName
    deleteRequest.joint_names = jointNames
    deleteRequest.action = deleteRequest.ACTION_DELETE_PATH
    deleteRequest.delete_id = pid
    rospy.wait_for_service(MANAGE_LIBRARY)
    response = manageClient(deleteRequest)
    if response.result == response.SUCCESS:
        rospy.loginfo("Manage library test: path deletion of path %i was successful" % (pid))
    else:
        rospy.loginfo("Manage library test: path deletion of path %i was not successful" % (pid))

def deleteLibrary(robotName, jointNames):
    manageClient = rospy.ServiceProxy(MANAGE_LIBRARY, ManagePathLibrary)
    deleteRequest = ManagePathLibraryRequest()
    deleteRequest.robot_name = robotName
    deleteRequest.joint_names = jointNames
    deleteRequest.action = deleteRequest.ACTION_DELETE_LIBRARY
    rospy.wait_for_service(MANAGE_LIBRARY)
    response = manageClient(deleteRequest)
    if response.result == response.SUCCESS:
        rospy.loginfo("Manage library test: path deletion of library for robot %s was successful" % (robotName))
    else:
        rospy.loginfo("Manage library test: path deletion of library for robot %s was not successful" % (robotName))

if __name__ == "__main__":
    rospy.init_node("manage_lib_tester")
    if len(sys.argv) >= 6 and sys.argv[1:4] == ["delete", "path", "right"]:
        deletePath(sys.argv[4], RIGHT_ARM_JOINT_NAMES, int(sys.argv[5]))
    elif len(sys.argv) >= 6 and sys.argv[1:4] == ["delete", "path", "left"]:
        deletePath(sys.argv[4], LEFT_ARM_JOINT_NAMES, int(sys.argv[5]))
    elif len(sys.argv) >= 5 and sys.argv[1:4] == ["delete", "library", "right"]:
        deleteLibrary(sys.argv[4], RIGHT_ARM_JOINT_NAMES)
    elif len(sys.argv) >= 5 and sys.argv[1:4] == ["delete", "library", "left"]:
        deleteLibrary(sys.argv[4], LEFT_ARM_JOINT_NAMES)
    else:
        rospy.loginfo("Manage library tester: Nothing to do")
