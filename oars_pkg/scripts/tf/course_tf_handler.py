#!/usr/bin/env python
import random

import itertools
import rospy

import tf2_ros
import tf2_geometry_msgs

from math import sin, cos

import json

from geometry_msgs.msg import Pose2D, Point, Quaternion, Pose, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

ft = 12 * 2.54 / 100


class CoursePoint(object):
    def __init__(self, x=0, y=0, theta=0, frame_id='course'):
        self.x = x
        self.y = y
        self.theta = theta
        self.frame_id = frame_id

        self.id = random.randint(0, 1 << 31)

    def from_json(self, json_object):
        # TODO: This probably isn't the best way to make an object with (kinda) multiple constructors
        self.x = json_object['x']
        self.y = json_object['y']
        self.theta = 0
        if 'theta' in json_object:
            self.theta = json_object['theta']

        if 'frame_id' in json_object:
            self.frame_id = json_object['frame_id']

        return self

    def as_pose2d(self):
        return Pose2D(x=self.x, y=self.y, theta=self.theta)

    def as_point(self):
        return Point(x=self.x, y=self.y)

    def as_pose(self):
        quat = CoursePoint.angle_to_quaternion(self.theta)
        point = self.as_point()
        return Pose(point=point, orientation=quat)

    @staticmethod
    def angle_to_quaternion(theta):
        return Quaternion(x=0, y=0, z=sin(theta / 2), w=-sin(theta / 2))


class Buoy(CoursePoint):
    color_lookup = {'red': ColorRGBA(r=1, a=1),
                    'green': ColorRGBA(g=1, a=1),
                    'blue': ColorRGBA(b=1, a=1),
                    'white': ColorRGBA(r=1, g=1, b=1, a=1)}

    sphere_size = 0.3
    cylinder_height = 1.0

    def __init__(self, x, y, type='cylinder', color='red', frame_id='course'):
        super(Buoy, self).__init__(x, y, frame_id=frame_id)

        assert (type in ['cylinder', 'sphere'])

        self.type = type
        self.color = self.color_lookup[color]

    def as_markers(self):
        """
        Creates and returns a list of Marker objects used to visualize this Buoy.
        In this case, that is a sphere or cylinder with the correct properties.
        :return: List of visualization_msgs/Marker object
        """
        marker = Marker()

        marker.header.frame_id = self.frame_id

        marker.ns = 'course'
        marker.id = self.id

        if self.type == 'sphere':
            marker.type = Marker.SPHERE
            marker.scale = Vector3(x=self.sphere_size, y=self.sphere_size, z=self.sphere_size)
        else:
            marker.type = Marker.CYLINDER
            marker.scale = Vector3(x=self.sphere_size, y=self.sphere_size, z=self.cylinder_height)

        marker.pose = self.as_pose()
        marker.color = self.color

        return [marker]


class NavigationGate(CoursePoint):
    """
    Represents a gate formed by two buoys as used in the navigation challenge.
    TODO: extend to also apply to buoy pair used at entrance/exit of speed challenge
    """

    def __init__(self, json_object, frame_id='course/navigation_entrance', gate_width=3 * ft):
        super(NavigationGate, self).__init__(frame_id=frame_id)
        self.from_json(json_object)

        self.leftBuoy = Buoy(0, gate_width / 2, 'cylinder', 'red', frame_id)
        self.rightBuoy = Buoy(0, -gate_width / 2, 'cylinder', 'green', frame_id)

    def as_markers(self):
        line = Marker()
        line.header.frame_id = self.frame_id
        line.type = Marker.LINE_STRIP
        line.ns = 'course'
        line.id = self.id
        line.color = ColorRGBA(r=1, b=1, g=1, a=0.5)
        line.points = [self.leftBuoy.as_point(), self.rightBuoy.as_point()]

        return [line] + self.leftBuoy.as_markers() + self.rightBuoy.as_markers()

    def as_transforms(self):
        # TODO: figure out publishing transforms
        pass


class NavigationChallenge(object):
    """
    Represents the navigation challenge, composed of two Navigation Gates.
    TODO: This should probably subclass some sort of Challenge class for cleanliness
    """

    def __init__(self, json_object):
        self.entrance_gate = NavigationGate(json_object['entrance_gate'], 'course/navigation_entrance')
        self.exit_gate = NavigationGate(json_object['exit_gate'], 'course/navigation_exit')

    def as_markers(self):
        return self.entrance_gate.as_markers() + self.exit_gate.as_markers()


class TFHandler(object):
    def __init__(self):
        super(TFHandler, self).__init__()

        rospy.init_node('course_tf_handler')

        self.name = 'Unnamed course'

        self.configFile = rospy.get_param('~config')

        self.challenges = []

    def loadConfig(self):
        with open(self.configFile) as file:
            data = json.load(file)
            if 'name' in data:
                self.name = data['name']

            if 'navigation_challenge' in data:
                self.challenges.append(NavigationChallenge(data['navigation_challenge']))
            print data
            print self.challenges

    def run(self):
        self.loadConfig()
        rospy.spin()


if __name__ == '__main__':
    TFHandler().run()
