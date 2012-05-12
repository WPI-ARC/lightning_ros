#!/usr/bin/env python
import roslib
roslib.load_manifest("lightning")
import rospy
import actionlib
import threading

from lightning.msg import PFSAction, PFSResult
from lightning.msg import StopPlanning, Float64Array

from tools.PathTools import PlanTrajectoryWrapper

PFS_NODE_NAME = "pfs_node";
STOP_PLANNER_NAME = "stop_pfs_planning";
STOP_PFS_NAME = "stop_all_pfs";

class PFSNode:
    def __init__(self):
        self.planTrajectoryWrapper = PlanTrajectoryWrapper("pfs")
        self.stopLock = threading.Lock()
        self.currentJointNames = []
        self.currentGroupName = ""
        self._setStopValue(True)
        self.PFSServer = actionlib.SimpleActionServer(PFS_NODE_NAME, PFSAction, execute_cb=self.getPath, auto_start=False)
        self.PFSServer.start()
        self.stopPFSSubscriber = rospy.Subscriber(STOP_PFS_NAME, StopPlanning, self.stopPFSPlanner)
        self.stopPFSPlannerPublisher = rospy.Publisher(STOP_PLANNER_NAME, StopPlanning)

    def _getStopValue(self):
        self.stopLock.acquire()
        ret = self.stop
        self.stopLock.release()
        return ret

    def _setStopValue(self, val):
        self.stopLock.acquire()
        self.stop = val
        self.stopLock.release()

    def _callPlanner(self, start, goal):
        plannerNumber = self.planTrajectoryWrapper.acquirePlanner()
        ret = self.planTrajectoryWrapper.planTrajectory(start, goal, plannerNumber, self.currentJointNames, self.currentGroupName)
        self.planTrajectoryWrapper.releasePlanner(plannerNumber)
        return ret

    def getPath(self, actionGoal):
        self._setStopValue(False)
        rospy.loginfo("PFS action server: PFS got an action goal")
        res = PFSResult()
        s, g = actionGoal.start, actionGoal.goal
        res.status.status = res.status.FAILURE
        self.currentJointNames = actionGoal.joint_names
        self.currentGroupName = actionGoal.group_name
        if not self._getStopValue():
            unfiltered = self._callPlanner(s, g)
            if unfiltered is None:
                self.PFSServer.set_succeeded(res)
                return
        else:
            rospy.loginfo("PFS action server: PFS was stopped before it started planning")
            self.PFSServer.set_succeeded(res)
            return

        if not self._getStopValue():
            res.status.status = res.status.SUCCESS
            res.path = [Float64Array(p) for p in unfiltered]
            self.PFSServer.set_succeeded(res)
            return
        else:
            rospy.loginfo("PFS action server: PFS found a path but RR succeeded first")
            self.PFSServer.set_succeeded(res)
            return

    def stopPFSPlanner(self, msg):
        self._setStopValue(True)
        rospy.loginfo("PFS action server: PFS node got a stop message")
        self.stopPFSPlannerPublisher.publish(msg)

if __name__ == "__main__":
    try:
        rospy.init_node("pfs_node")
        PFSNode()
        rospy.loginfo("PFSNode is ready")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass;


