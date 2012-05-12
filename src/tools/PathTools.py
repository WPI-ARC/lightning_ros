import roslib
roslib.load_manifest("lightning")
import rospy

import threading
import sys

from lightning.msg import Float64Array, Float64Array2D, DrawPoints
from lightning.srv import CollisionCheck, CollisionCheckRequest, PathShortcut, PathShortcutRequest
from arm_navigation_msgs.srv import GetMotionPlan, GetMotionPlanRequest
from arm_navigation_msgs.msg import JointConstraint

COLLISION_CHECK = "collision_check"
SHORTCUT_PATH_NAME = "shortcut_path"
DISPLAY_POINTS = "draw_points"

class PlanTrajectoryWrapper:

    def __init__(self, nodeType, numPlanners=1):
        self.PLANNERS = ["/%s_planner_node%i/%s_planner%i" % (nodeType, i, nodeType, i) for i in xrange(numPlanners)]
        rospy.loginfo("Initializaing %i planners for %s" % (numPlanners, nodeType))
        self.plannerTime = float(rospy.get_param("allowed_planning_time"))
        self.plannerAvailable = [True for i in xrange(numPlanners)]
        self.plannerLock = threading.Lock()
        self.releasedEvent = threading.Event()
        self.releasedEvent.set()

    #need to call acquirePlanner before calling planTrajectory
    def acquirePlanner(self):
        plannerNumber = self._waitForPlanner()
        while plannerNumber == -1:
            self.releasedEvent.wait()
            plannerNumber = self._waitForPlanner()
        return plannerNumber
   
    #need to call releasePlanner after done calling planTrajectory
    def releasePlanner(self, index):
        self.plannerLock.acquire()
        self.plannerAvailable[index] = True
        self.releasedEvent.set()
        self.plannerLock.release()

    def _waitForPlanner(self):
        self.plannerLock.acquire()
        acquiredPlanner = -1
        for i, val in enumerate(self.plannerAvailable):
            if val:
                self.plannerAvailable[i] = False
                if not any(self.plannerAvailable):
                    self.releasedEvent.clear()
                acquiredPlanner = i
                break
        self.plannerLock.release()
        return acquiredPlanner

    #planner to get new trajectory from start_point to goal_point
    #plannerNumber is the number received from acquirePlanner
    def planTrajectory(self, start_point, goal_point, plannerNumber, joint_names, groupName, plannerConfigName="RRTConnectkConfig1"):
        plannerClient = rospy.ServiceProxy(self.PLANNERS[plannerNumber], GetMotionPlan)
        rospy.loginfo("Plan Trajectory Wrapper: got a planTrajectory request for %s with start = %s and goal = %s" % (self.PLANNERS[plannerNumber], start_point, goal_point))
        req = GetMotionPlanRequest()
        req.motion_plan_request.workspace_parameters.workspace_region_pose.header.stamp = rospy.get_rostime()
        req.motion_plan_request.group_name = groupName
        req.motion_plan_request.num_planning_attempts = 1
        req.motion_plan_request.allowed_planning_time = rospy.Duration(self.plannerTime)
        req.motion_plan_request.planner_id = plannerConfigName #using RRT planner by default

        req.motion_plan_request.start_state.joint_state.header.stamp = rospy.get_rostime()
        req.motion_plan_request.start_state.joint_state.name = joint_names
        req.motion_plan_request.start_state.joint_state.position = start_point

        req.motion_plan_request.goal_constraints.joint_constraints = []
        for i in xrange(len(joint_names)):
            tempConstraint = JointConstraint()
            tempConstraint.joint_name = joint_names[i]
            tempConstraint.position = goal_point[i]
            req.motion_plan_request.goal_constraints.joint_constraints.append(tempConstraint)

        #call the planner
        rospy.wait_for_service(self.PLANNERS[plannerNumber])
        rospy.loginfo("Plan Trajectory Wrapper: sent request to service %s" % plannerClient.resolved_name)
        try:
            response = plannerClient(req)
        except rospy.ServiceException, e:
            rospy.loginfo("Plan Trajectory Wrapper: service call failed: %s"%e)
            return None

        rospy.loginfo("Plan Trajectory Wrapper: %s returned" % (self.PLANNERS[plannerNumber]))
        if response.error_code.val == response.error_code.SUCCESS:
            #rospy.loginfo([pt.time_from_start for pt in response.trajectory.joint_trajectory.points])
            return [pt.positions for pt in response.trajectory.joint_trajectory.points]
        else:
            rospy.loginfo("Plan Trajectory Wrapper: service call to %s was unsuccessful" % plannerClient.resolved_name)
            return None

class ShortcutPathWrapper:

    def shortcutPath(self, origPath, groupName):
        shortcutPathClient = rospy.ServiceProxy(SHORTCUT_PATH_NAME, PathShortcut)
        shortcutReq = PathShortcutRequest()
        shortcutReq.path = [Float64Array(p) for p in origPath]
        shortcutReq.group_name = groupName
        rospy.wait_for_service(SHORTCUT_PATH_NAME)
        response = shortcutPathClient(shortcutReq)
        return [p.values for p in response.new_path]

class InvalidSectionWrapper:

    def getInvalidSectionsForPath(self, origPath, groupName):
        section = self.getInvalidSectionsForPaths([origPath], groupName)
        if len(section) > 0:
            return section[0]
        else:
            return None

    def getInvalidSectionsForPaths(self, orig_paths, groupName):
        collisionCheckClient = rospy.ServiceProxy(COLLISION_CHECK, CollisionCheck)
        ccReq = CollisionCheckRequest();
        ccReq.paths = [Float64Array2D([Float64Array(point) for point in path]) for path in orig_paths];
        ccReq.group_name = groupName
        rospy.loginfo("Plan Trajectory Wrapper: sending request to collision checker")
        rospy.wait_for_service(COLLISION_CHECK)
        response = collisionCheckClient(ccReq);
        return [[sec.values for sec in individualPathSections.points] for individualPathSections in response.invalid_sections];

class DrawPointsWrapper:

    #point colors
    WHITE = (1.0, 1.0, 1.0)
    BLACK = (0.0, 0.0, 0.0)
    RED = (1.0, 0.0, 0.0)
    GREEN = (0.0, 1.0, 0.0)
    BLUE = (0.0, 0.0, 1.0)
    MAGENTA = (1.0, 0.0, 1.0)
    YELLOW = (1.0, 1.0, 0.0)
    GREENBLUE = (0.0, 1.0, 1.0)
    
    #point types
    ANGLES = "angles"
    POSES = "poses"
    
    def __init__(self):
        self.displayPointsPublisher = rospy.Publisher(DISPLAY_POINTS, DrawPoints)

    def drawPoints(self, path, modelGroupName, pointGroupName, pointType, rgb, displayDensity, pointRadius=0.03):
        drawMessage = DrawPoints()
        drawMessage.points = [Float64Array(p) for p in path]
        drawMessage.model_group_name = modelGroupName
        drawMessage.point_group_name = pointGroupName
        drawMessage.point_type = drawMessage.POINT_TYPE_ANGLES if pointType == DrawPointsWrapper.ANGLES else drawMessage.POINT_TYPE_POSES
        drawMessage.display_density = displayDensity
        drawMessage.red, drawMessage.green, drawMessage.blue = rgb
        drawMessage.action = drawMessage.ACTION_ADD
        drawMessage.point_radius = pointRadius
        self.displayPointsPublisher.publish(drawMessage)

    def clearPoints(self):
        drawMessage = DrawPoints()
        drawMessage.action = drawMessage.ACTION_CLEAR
        self.displayPointsPublisher.publish(drawMessage)

if __name__ == "__main__":
    if len(sys.argv) == 8:
        isw = InvalidSectionWrapper()
        path = [float(sys.argv[i]) for i in xrange(1, len(sys.argv))]
        print isw.getInvalidSectionsForPath([path])
