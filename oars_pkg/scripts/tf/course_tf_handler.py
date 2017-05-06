#!/usr/bin/env python
import json
import random

import math
import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import Pose2D, Point, Pose, Vector3, Transform, TransformStamped, Quaternion
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

ft = 12 * 2.54 / 100


class CoursePoint(object):
    """
    Base class to represent a 2D pose on the course.
    """

    def __init__(self, x=0, y=0, theta=0, frame_id='course'):
        """
        :param x: x-coordinate of point in frame frame_id
        :param y: x-coordinate of point in frame frame_id
        :param theta: angle of point, in radians
        :param frame_id: Name of the coordinate frame in which the CoursePoint is defined
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.frame_id = frame_id

        # TODO: make IDs permanent across script restarts
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
        return Pose(position=point, orientation=quat)

    def as_transforms(self, child_frame=None):
        """
        Calculates the Transform associated with this point
        :param child_frame: The frame whose origin is attached to this point
        :return: list of geometry_msgs/TransformStamped
        """
        pose = self.as_pose()
        # TODO: consider publishing the transform dated in the future
        header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        transform = Transform(translation=pose.position, rotation=pose.orientation)
        return [TransformStamped(header=header, child_frame_id=child_frame, transform=transform)]

    @staticmethod
    def angle_to_quaternion(theta):
        quat = tf_conversions.transformations.quaternion_from_euler(0, 0, theta * math.pi / 180)
        return Quaternion(*quat)


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
        marker.pose = self.as_pose()

        if self.type == 'sphere':
            marker.type = Marker.SPHERE
            marker.scale = Vector3(x=self.sphere_size, y=self.sphere_size, z=self.sphere_size)
            marker.pose.position.z += self.sphere_size / 2
        else:
            marker.type = Marker.CYLINDER
            marker.scale = Vector3(x=self.sphere_size, y=self.sphere_size, z=self.cylinder_height)
            marker.pose.position.z += self.cylinder_height / 2

        marker.color = self.color

        return [marker]


class Gate(CoursePoint):
    """
    Represents a gate formed by two buoys as used in the navigation challenge and the speed challenge.
    """

    def __init__(self, json_object,
                 frame_id='course', child_frame='course/navigation_entrance',
                 gate_width=3 * ft, shape='cylinder'):
        super(Gate, self).__init__(frame_id=frame_id)
        self.from_json(json_object)

        self.child_frame = child_frame

        self.leftBuoy = Buoy(0, gate_width / 2, shape, 'red', child_frame)
        self.rightBuoy = Buoy(0, -gate_width / 2, shape, 'green', child_frame)

    def as_markers(self):
        line = Marker()
        line.header.frame_id = self.child_frame
        line.type = Marker.LINE_STRIP
        line.ns = 'course'
        line.id = self.id
        line.color = ColorRGBA(r=0, b=1, g=1, a=0.9)
        line.scale = Vector3(0.05, 0.05, 0.05)
        line.points = [self.leftBuoy.as_point(), self.rightBuoy.as_point()]

        return [line] + self.leftBuoy.as_markers() + self.rightBuoy.as_markers()


class NavigationChallenge(object):
    """
    Represents the navigation challenge, composed of two Navigation Gates.
    """

    # TODO: This should probably subclass some sort of Challenge class for cleanliness

    def __init__(self, json_object):
        # TODO: store this information inside the Gate object somewhere
        self.entrance_gate = Gate(json_object['entrance_gate'], 'course', child_frame='course/navigation_entrance')
        self.exit_gate = Gate(json_object['exit_gate'], 'course', child_frame='course/navigation_exit')

    def as_markers(self):
        return self.entrance_gate.as_markers() + self.exit_gate.as_markers()

    def as_transforms(self):
        return (
            self.entrance_gate.as_transforms('course/navigation_entrance') +
            self.exit_gate.as_transforms('course/navigation_exit')
        )


class SpeedChallenge(object):
    """
    Represents the Speed challenge, composed of one Gate and one Buoy.
    """

    # TODO: This should probably subclass some sort of Challenge class for cleanliness

    def __init__(self, json_object):
        self.gate = Gate(json_object['gate'], 'course', child_frame='course/speed_gate',
                         shape='sphere', gate_width=5*ft)
        self.buoy = Buoy(0, 0, type='sphere', color='blue')
        self.buoy.from_json(json_object['buoy'])

    def as_markers(self):
        return self.gate.as_markers() + self.buoy.as_markers()

    def as_transforms(self):
        return (
            self.gate.as_transforms('course/speed_gate') +
            self.buoy.as_transforms('course/speed_buoy')
        )


class TFHandler(object):
    def __init__(self):
        super(TFHandler, self).__init__()

        rospy.init_node('course_tf_handler')

        self.marker_pub = rospy.Publisher('/course_markers', MarkerArray, queue_size=10)
        self.tf_pub = tf2_ros.TransformBroadcaster()

        self.name = 'Unnamed course'
        self.challenges = []
        self.configFile = rospy.get_param('~config')

    def loadConfig(self):
        with open(self.configFile) as file:
            data = json.load(file)
            if 'name' in data:
                self.name = data['name']

            if 'navigation_challenge' in data:
                self.challenges.append(NavigationChallenge(data['navigation_challenge']))

            if 'speed_challenge' in data:
                self.challenges.append(SpeedChallenge(data['speed_challenge']))

    def as_markers(self):
        markers = []
        for challenge in self.challenges:
            markers.extend(challenge.as_markers())
        return markers

    def as_transforms(self):
        transforms = []
        for challenge in self.challenges:
            transforms.extend(challenge.as_transforms())
        return transforms

    def run(self):
        self.loadConfig()

        print 'Running'

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.marker_pub.publish(MarkerArray(self.as_markers()))
            self.tf_pub.sendTransform(self.as_transforms())

            r.sleep()


if __name__ == '__main__':
    TFHandler().run()
