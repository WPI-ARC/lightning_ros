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

from visualization_msgs.msg import Marker, MarkerArray
from lightning.msg import Status
from lightning.msg import DrawPoints
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPlanningSceneRequest, GetPlanningScene, GetPositionFKRequest, GetPositionFK, GetKinematicSolverInfo, GetKinematicSolverInfoRequest
from geometry_msgs.msg import Point

DRAW_POINTS = "draw_points"
MARKER_SUBSCRIBER_NAME = "visualization_marker"

class PointDrawer:
    """
      This handles the direct drawing of points in RViz by listening to the
        DRAW_POINTS topic and then publishing a MarkerArray which can be
        visualized in RViz.
    """
    def __init__(self):
        self.current_points = dict()
        self.draw_subscriber = rospy.Subscriber(DRAW_POINTS, DrawPoints, self._do_draw_action)
        self.marker_publisher = rospy.Publisher(MARKER_SUBSCRIBER_NAME, MarkerArray, queue_size=10)

    def _do_draw_action(self, msg):
        points = []
        edges = []
        if msg.action == msg.ACTION_ADD:
            if msg.model_group_name in ["right_arm", "left_arm"]:
                rospy.loginfo("Point drawer: got a set of %i points to draw for %s for %s" % (len(msg.points), msg.model_group_name, msg.point_group_name))
                if len(msg.points) > 0:
                    if msg.point_type == msg.POINT_TYPE_ANGLES:
                        point_id = 0
                        jump_num = int(1.0/msg.display_density)
                        for i in xrange(0, len(msg.points), jump_num):
                            point = self._get_coordinates(msg.points[i].values, msg.model_group_name)
                            if point is None:
                                return
                            else:
                                points.append(point)
                        if (len(msg.points)-1) % jump_num != 0: #add goal
                            point = self._get_coordinates(msg.points[-1].values, msg.model_group_name)
                            if point is None:
                                return
                            else:
                                points.append(point)
                        self._publish_points(points, msg.point_group_name, msg.red, msg.green, msg.blue, msg.point_radius)
                        self.current_points[msg.point_group_name] = len(points)
                    elif msg.point_type == msg.POINT_TYPE_POSES:
                        point_id = 0
                        jump_num = int(1.0/msg.display_density)
                        for i in xrange(0, len(msg.points), jump_num):
                            point = msg.points[i].values
                            points.append(point)
                        if (len(msg.points)-1) % jump_num != 0: #add goal
                            point = msg.points[i].values
                            points.append(point)
                    else:
                        rospy.loginfo("Point drawer: point type not set")
                        return

                    self.current_points[msg.point_group_name] = len(points)

                    if len(msg.edges) > 0:
                        edges = [endpoint_list.values for endpoint_list in msg.edges]
                        self.current_points[msg.point_group_name] += 1

                    self._publish_points(points, msg.point_group_name, msg.red, msg.green, msg.blue, msg.point_radius, edges)
            else:
                rospy.loginfo("Point drawer: got invalid group name: %s" % (msg.model_group_name))
        elif msg.action == msg.ACTION_CLEAR:
            self._clear_points()
        else:
            rospy.loginfo("Point drawer: action not set")
            return
        return

    def _clear_points(self):
        marker_arr = MarkerArray()
        for point_group_name in self.current_points:
            for i in xrange(self.current_points[point_group_name]):
                marker_arr.markers.append(self._create_clear_marker(point_group_name, i))
        self.marker_publisher.publish(marker_arr)
        self.current_points = dict()

    def _publish_points(self, points, point_group_name, red, green, blue, point_radius, edges=[]):
        marker_arr = MarkerArray()
        if point_radius > 0:
            for i, pt in enumerate(points):
                marker_arr.markers.append(self._create_add_point(pt, point_group_name, i, red, green, blue, point_radius))
        if len(edges) > 0:
            marker_arr.markers.append(self._create_add_line(points, edges, point_group_name, len(points), red, green, blue))
        self.marker_publisher.publish(marker_arr)

    def _create_add_point(self, coords, point_group_name, id, red, green, blue, point_radius):
        marker = Marker()
        marker.header.frame_id = "odom_combined"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = point_group_name
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
        marker.scale.x = point_radius
        marker.scale.y = point_radius
        marker.scale.z = point_radius

        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration()
        return marker

    def _create_add_line(self, points, edges, point_group_name, id, red, green, blue):
        all_points = []
        for i, end_points in enumerate(edges):
            for endpoint_index in end_points:
                pt1, pt2 = Point(), Point()
                pt1.x, pt1.y, pt1.z = points[i][0], points[i][1], points[i][2]
                all_points.append(pt1)
                pt2.x, pt2.y, pt2.z = points[endpoint_index][0], points[endpoint_index][1], points[endpoint_index][2]
                all_points.append(pt2)

        marker = Marker()
        marker.header.frame_id = "odom_combined"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = point_group_name
        marker.id = id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = all_points

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

    def _create_clear_marker(self, point_group_name, id):
        marker = Marker()
        marker.header.frame_id = "odom_combined"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = point_group_name
        marker.id = id
        marker.action = Marker.DELETE
        return marker

    def _get_coordinates(self, point, arm):
        if arm not in ["right_arm", "left_arm"]: #can only draw points for pr2 arms
            return None
        FK_NAME = "/compute_fk"
        FK_INFO_NAME = "/pr2_%s_kinematics/get_fk_solver_info" % (arm)

        info_client = rospy.ServiceProxy(FK_INFO_NAME, GetKinematicSolverInfo)
        info_request = GetKinematicSolverInfoRequest()
        rospy.wait_for_service(FK_INFO_NAME)
        info_response = info_client(info_request)

        fk_client = rospy.ServiceProxy(FK_NAME, GetPositionFK)
        fk_request = GetPositionFKRequest()
        fk_request.header.frame_id = "odom_combined"
        fk_request.fk_link_names.append("%s_wrist_roll_link" % (arm[0]))

        fk_request.robot_state.joint_state.name = info_response.kinematic_solver_info.joint_names
#fk_request.robot_state = self._get_robot_state()
        fk_request.robot_state.joint_state.position = []
        for i in xrange(len(info_response.kinematic_solver_info.joint_names)):
            fk_request.robot_state.joint_state.position.append(point[i])
        rospy.wait_for_service(FK_NAME)
        fk_solve_response = fk_client(fk_request)
        if(fk_solve_response.error_code.val == fk_solve_response.error_code.SUCCESS):
            position = fk_solve_response.pose_stamped[0].pose.position
            return (position.x, position.y, position.z)
        else:
            rospy.loginfo("Forward kinematics service call failed")
            return None

    def _get_robot_state(self):
      GET_PLANNING_SCENE_NAME = "/get_planning_scene"
      rospy.wait_for_service(GET_PLANNING_SCENE_NAME)
      robot_state_client = rospy.ServiceProxy(GET_PLANNING_SCENE_NAME, GetPlanningScene)
      robot_state_req = GetPlanningSceneRequest()
      robot_state_req.components.components = robot_state_req.components.ROBOT_STATE
      robot_state = robot_state_client(robot_state_req).scene.robot_state
      return robot_state

if __name__ == "__main__":
    try:
        rospy.init_node("point_drawer")
        PointDrawer()
        rospy.loginfo("Point drawer: ready")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass;
