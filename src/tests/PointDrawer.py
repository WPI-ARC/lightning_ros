#!/usr/bin/env python
import roslib
roslib.load_manifest("lightning")
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from lightning.msg import Status
from lightning.msg import DrawPoints
from kinematics_msgs.srv import GetKinematicSolverInfo, GetKinematicSolverInfoRequest, GetPositionFK, GetPositionFKRequest
from geometry_msgs.msg import Point

DRAW_POINTS = "draw_points"
MARKER_SUBSCRIBER_NAME = "visualization_marker_array"

class PointDrawer:
    def __init__(self):
        self.currentPoints = dict()
        self.drawSubscriber = rospy.Subscriber(DRAW_POINTS, DrawPoints, self.doDrawAction)
        self.markerPublisher = rospy.Publisher(MARKER_SUBSCRIBER_NAME, MarkerArray)

    def doDrawAction(self, msg):
        points = []
        edges = []
        if msg.action == msg.ACTION_ADD:
            if msg.model_group_name in ["right_arm", "left_arm"]:
                rospy.loginfo("Point drawer: got a set of %i points to draw for %s" % (len(msg.points), msg.model_group_name))
                if len(msg.points) > 0:
                    if msg.point_type == msg.POINT_TYPE_ANGLES:
                        pointId = 0
                        jumpNum = int(1.0/msg.display_density)
                        for i in xrange(0, len(msg.points), jumpNum):
                            point = self._getCoordinates(msg.points[i].values, msg.model_group_name)
                            if point is None:
                                return
                            else:
                                points.append(point)
                        if (len(msg.points)-1) % jumpNum != 0: #add goal
                            point = self._getCoordinates(msg.points[-1].values, msg.model_group_name)
                            if point is None:
                                return
                            else:
                                points.append(point)
                        self._publishPoints(points, msg.point_group_name, msg.red, msg.green, msg.blue, msg.point_radius)
                        self.currentPoints[msg.point_group_name] = len(points)
                    elif msg.point_type == msg.POINT_TYPE_POSES:
                        pointId = 0
                        jumpNum = int(1.0/msg.display_density)
                        for i in xrange(0, len(msg.points), jumpNum):
                            point = msg.points[i].values
                            points.append(point)
                        if (len(msg.points)-1) % jumpNum != 0: #add goal
                            point = msg.points[i].values
                            points.append(point)
                    else:
                        rospy.loginfo("Point drawer: point type not set")
                        return
                    
                    self.currentPoints[msg.point_group_name] = len(points)
                    
                    if len(msg.edges) > 0:
                        edges = [endPointList.values for endPointList in msg.edges]
                        self.currentPoints[msg.point_group_name] += 1

                    self._publishPoints(points, msg.point_group_name, msg.red, msg.green, msg.blue, msg.point_radius, edges)
            else:
                rospy.loginfo("Point drawer: got invalid group name: %s" % (msg.model_group_name))
        elif msg.action == msg.ACTION_CLEAR:
            self._clearPoints() 
        else:
            rospy.loginfo("Point drawer: action not set")
            return
        return

    def _clearPoints(self):
        markerArr = MarkerArray()
        for groupName in self.currentPoints:
            for i in xrange(self.currentPoints[groupName]):
                markerArr.markers.append(self._createClearMarker(groupName, i))
        self.markerPublisher.publish(markerArr)
        self.currentPoints = dict()

    def _publishPoints(self, points, groupName, red, green, blue, pointRadius, edges=[]):
        markerArr = MarkerArray()
        if pointRadius > 0:
            for i, pt in enumerate(points):
                markerArr.markers.append(self._createAddPoint(pt, groupName, i, red, green, blue, pointRadius))
        if len(edges) > 0:
            markerArr.markers.append(self._createAddLine(points, edges, groupName, len(points), red, green, blue))
        self.markerPublisher.publish(markerArr)

    def _createAddPoint(self, coords, groupName, id, red, green, blue, pointRadius):
        marker = Marker()
        marker.header.frame_id = "odom_combined"
        marker.header.stamp = rospy.get_rostime()

        marker.ns = groupName
        marker.id = id

        marker.type = Marker.SPHERE

        marker.action = Marker.ADD

        marker.pose.position.x = coords[0]
        marker.pose.position.y = coords[1]
        marker.pose.position.z = coords[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = pointRadius
        marker.scale.y = pointRadius
        marker.scale.z = pointRadius

        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration()
        return marker
    
    def _createAddLine(self, points, edges, groupName, id, red, green, blue):
        allPoints = []
        for i, endPoints in enumerate(edges):
            for endPointIndex in endPoints:
                pt1, pt2 = Point(), Point()
                pt1.x, pt1.y, pt1.z = points[i][0], points[i][1], points[i][2]
                allPoints.append(pt1)
                pt2.x, pt2.y, pt2.z = points[endPointIndex][0], points[endPointIndex][1], points[endPointIndex][2]
                allPoints.append(pt2)
    
        marker = Marker()
        marker.header.frame_id = "odom_combined"
        marker.header.stamp = rospy.get_rostime()

        marker.ns = groupName
        marker.id = id

        marker.type = Marker.LINE_STRIP

        marker.action = Marker.ADD

        marker.points = allPoints

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.003
        
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration()
        return marker

    def _createClearMarker(self, groupName, id):
        marker = Marker()
        marker.header.frame_id = "odom_combined"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = groupName
        marker.id = id
        marker.action = Marker.DELETE
        return marker

    def _getCoordinates(self, point, arm):
        if arm not in ["right_arm", "left_arm"]: #can only draw points for pr2 arms
            return None
        FK_INFO_NAME = "pr2_%s_kinematics/get_fk_solver_info" % (arm)
        FK_NAME = "pr2_%s_kinematics/get_fk" % (arm)

        infoClient = rospy.ServiceProxy(FK_INFO_NAME, GetKinematicSolverInfo)
        infoRequest = GetKinematicSolverInfoRequest()
        rospy.wait_for_service(FK_INFO_NAME)
        infoResponse = infoClient(infoRequest)
        
        fkClient = rospy.ServiceProxy(FK_NAME, GetPositionFK)
        fkRequest = GetPositionFKRequest()
        fkRequest.header.frame_id = "odom_combined"
        fkRequest.fk_link_names.append("%s_wrist_roll_link" % (arm[0]))

        fkRequest.robot_state.joint_state.name = infoResponse.kinematic_solver_info.joint_names
        fkRequest.robot_state.joint_state.position = []
        for i in xrange(len(infoResponse.kinematic_solver_info.joint_names)):
            fkRequest.robot_state.joint_state.position.append(point[i])
        rospy.wait_for_service(FK_NAME)
        fkSolveResponse = fkClient(fkRequest)
        if(fkSolveResponse.error_code.val == fkSolveResponse.error_code.SUCCESS):
            #rospy.loginfo("Point drawer: forward kinematics was successful: pose is (%f, %f, %f)", fkSolveResponse.pose_stamped[0].pose.position.x, fkSolveResponse.pose_stamped[0].pose.position.y, fkSolveResponse.pose_stamped[0].pose.position.z);    
            position = fkSolveResponse.pose_stamped[0].pose.position
            return (position.x, position.y, position.z)
        else:
            rospy.loginfo("Forward kinematics service call failed")
            return None

if __name__ == "__main__":
    try:
        rospy.init_node("point_drawer")
        PointDrawer()
        rospy.loginfo("Point drawer: ready")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass;
