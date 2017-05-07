#!/usr/bin/env python
import json
import random

import math
import rospy
import tf2_geometry_msgs
import tf2_ros
import tf_conversions
from geometry_msgs.msg import Pose2D, Point, Pose, Vector3, Transform, TransformStamped, Quaternion, Point32, \
    PointStamped
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud

# Define units for convenience later
inch = 2.54 / 100
foot = 12 * inch

tf_buff = tf2_ros.Buffer()


class CoursePoint(object):
    """
    Base class to represent a 2D pose on the course. It has helper functions
    to convert to various other data types that are useful in different contexts.
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
        """
        Updates the position and (if available) orientation of this Point using the data in the provided
        JSON blob. This should be called after the CoursePoint has been initialized.
        :param (dict) json_object:
        :return: self
        """
        # TODO: This probably isn't the best way to make an object with (kinda) multiple constructors
        self.x = json_object['x']
        self.y = json_object['y']
        self.theta = 0
        if 'theta' in json_object:
            self.theta = json_object['theta'] * math.pi/180

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
        Calculates the Transform associated with this point, in this case a list of length 1
        :param child_frame: The frame whose origin is attached to this point
        :return list of TransformStamped: Transforms associated with this point
        """
        pose = self.as_pose()
        # TODO: consider publishing the transform dated in the future
        header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        transform = Transform(translation=pose.position, rotation=pose.orientation)
        return [TransformStamped(header=header, child_frame_id=child_frame, transform=transform)]

    @staticmethod
    def angle_to_quaternion(theta):
        quat = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        return Quaternion(*quat)


class Buoy(CoursePoint):
    color_lookup = {'red': ColorRGBA(r=1, a=1),
                    'green': ColorRGBA(g=1, a=1),
                    'blue': ColorRGBA(b=1, a=1),
                    'white': ColorRGBA(r=1, g=1, b=1, a=1)}

    cylinder_height = 1.0

    def __init__(self, x, y, kind='cylinder', color='red', frame_id='course', sphere_size=0.3):
        super(Buoy, self).__init__(x, y, frame_id=frame_id)

        assert (kind in ['cylinder', 'sphere'])

        self.kind = kind
        self.color = self.color_lookup[color]
        self.sphere_size = sphere_size

    def as_markers(self):
        """
        Creates and returns a list of Marker objects used to visualize this Buoy.
        In this case, that is a sphere or cylinder with the correct properties.
        :return list of Marker: Markers associated with this object
        """
        marker = Marker()

        marker.header.frame_id = self.frame_id

        marker.ns = 'course'
        marker.id = self.id
        marker.pose = self.as_pose()

        if self.kind == 'sphere':
            marker.type = Marker.SPHERE
            marker.scale = Vector3(x=self.sphere_size, y=self.sphere_size, z=self.sphere_size)
            marker.pose.position.z += self.sphere_size / 2
        else:
            marker.type = Marker.CYLINDER
            marker.scale = Vector3(x=self.sphere_size, y=self.sphere_size, z=self.cylinder_height)
            marker.pose.position.z += self.cylinder_height / 2

        marker.color = self.color

        return [marker]

    def update_detection(self, msg, uncertainty=3.0):
        """
        Updates the position of the buoy using the detections provided in msg.
        Note that this does not ingest the raw points from the lidar, but the candidate buoy locations
        from the processing script.
        :param (PointCloud) msg: A list of possible buoy detections from the LIDAR processor
        :param (float) uncertainty: How close to the buoy a detection must be to have any effect
        :return bool: True if the position of the buoy was updated, False otherwise.
        """

        transform = tf_buff.lookup_transform(self.frame_id, msg.header.frame_id, msg.header.stamp)

        valid_detections = []
        for p in msg.points:
            p = Point(p.x, p.y, 0)
            local_point = tf2_geometry_msgs.do_transform_point(p, transform).point

            dist = math.sqrt((local_point.x - self.x) ** 2 + (local_point.y - self.y) ** 2)
            if dist < uncertainty:
                valid_detections.append(local_point)

        if len(valid_detections) == 1:
            # Update the position of the Buoy based on the detection
            p = valid_detections[0]

            # TODO: Do something more intelligent to be less likely to jump around at spurious detections
            self.x = p.x
            self.y = p.y

            return True

        else:
            return False


class Gate(CoursePoint):
    """
    Represents a gate formed by two buoys as used in the navigation challenge or the speed challenge.
    """

    def __init__(self, json_object,
                 frame_id='course', child_frame='course/navigation_entrance',
                 gate_width=3 * foot, shape='cylinder'):
        """
        :param (dict) json_object:
        :param (string) frame_id: The tf frame in which the coordinates of the Gate are defined
        :param (string) child_frame: The tf frame locked to the center of the Gate
        :param (float) gate_width: The approximate centerline distance between the two buoys, in meters
        :param (string) shape: One of "cylinder" or "sphere" depending on the type of the buoys
        """
        super(Gate, self).__init__(frame_id=frame_id)
        self.from_json(json_object)

        self.child_frame = child_frame
        self.gate_width = gate_width

        self.leftBuoy = Buoy(0, gate_width / 2, shape, 'red', child_frame)
        self.rightBuoy = Buoy(0, -gate_width / 2, shape, 'green', child_frame)

    def as_markers(self):
        """
        Returns three markers: one cylinder or sphere for each buoy, and a line connecting them.
        :return List of Marker
        """
        line = Marker()
        line.header.frame_id = self.child_frame
        line.type = Marker.LINE_STRIP
        line.ns = 'course'
        line.id = self.id
        line.color = ColorRGBA(r=0, b=1, g=1, a=0.9)
        line.scale = Vector3(0.05, 0.05, 0.05)
        line.points = [self.leftBuoy.as_point(), self.rightBuoy.as_point()]

        return [line] + self.leftBuoy.as_markers() + self.rightBuoy.as_markers()

    def update_detection(self, msg, uncertainty=3.0):
        """
        Updates the position and orientation of the gate using the detections provided in msg.
        Note that this does not ingest the raw points from the lidar, but the candidate buoy locations
        from the processing script.
        :param (PointCloud) msg: A list of possible buoy detections from the LIDAR processor
        :param (float) uncertainty: How close to the buoy a detection must be to have any effect
        :return bool: True if the position of the buoy was updated, False otherwise.
        """

        transform = tf_buff.lookup_transform(self.frame_id, msg.header.frame_id, msg.header.stamp)

        valid_detections = []
        for p in msg.points:
            p = Point(p.x, p.y, 0)
            local_point = tf2_geometry_msgs.do_transform_point(p, transform).point

            # TODO Check for diatance from each of the two buoys instead of this
            dist = math.sqrt((local_point.x - self.x) ** 2 + (local_point.y - self.y) ** 2)
            if dist < uncertainty + self.gate_width / 2:
                valid_detections.append(local_point)

        if len(valid_detections) == 2:
            # Update the position of the Gate based on the detections
            p1, p2 = valid_detections

            # TODO: Do something more intelligent to be less likely to jump around at spurious detections
            self.x = (p1.x + p2.x)/2
            self.y = (p1.y + p2.y)/2

            # The buoys are offset 90 degrees from the x-axis
            currentTheta = self.theta + math.pi/2
            # The new buoy detections also form some angle
            newTheta = math.atan2(p2.y-p1.y, p2.x-p1.x)

            angleDelta = newTheta - currentTheta

            # Make sure that the left and right buoys don't switch places completely.
            while angleDelta < -math.pi/2:
                angleDelta += math.pi
            while angleDelta > math.pi/2:
                angleDelta -= math.pi

            # Update the angle
            self.theta += angleDelta

            return True

        else:
            return False


class NavigationChallenge(object):
    """
    Represents the navigation challenge setup, composed of two Navigation Gates.
    """

    # TODO: This should probably subclass some sort of Challenge class for cleanliness

    def __init__(self, json_object):
        # TODO: store the child frame information inside the Gate object to prevent duplication
        self.entrance_gate = Gate(json_object['entrance_gate'], 'course', child_frame='course/navigation_entrance')
        self.exit_gate = Gate(json_object['exit_gate'], 'course', child_frame='course/navigation_exit')

    def as_markers(self):
        return self.entrance_gate.as_markers() + self.exit_gate.as_markers()

    def as_transforms(self):
        return (
            self.entrance_gate.as_transforms(self.entrance_gate.child_frame) +
            self.exit_gate.as_transforms(self.exit_gate.child_frame)
        )


class SpeedChallenge(object):
    """
    Represents the Speed challenge, composed of one Gate and one Buoy.
    """

    def __init__(self, json_object):
        self.gate = Gate(json_object['gate'], 'course', child_frame='course/speed_gate',
                         shape='sphere', gate_width=5 * foot)

        if 'theta' not in json_object['buoy'] and 'theta' in json_object['gate']:
            json_object['buoy']['theta'] = json_object['gate']['theta']

        self.buoy = Buoy(0, 0, kind='sphere', color='blue')
        self.buoy.from_json(json_object['buoy'])

    def as_markers(self):
        return self.gate.as_markers() + self.buoy.as_markers()

    def as_transforms(self):
        return (
            self.gate.as_transforms('course/speed_gate') +
            self.buoy.as_transforms('course/speed_buoy')
        )


class TFHandler(object):
    """
    Class for the ROS node associated with managing and handling tf frames for RoboBoat.
    """

    def __init__(self):
        super(TFHandler, self).__init__()

        rospy.init_node('course_tf_handler')

        config_file = rospy.get_param('~config')

        self.marker_pub = rospy.Publisher('/course_markers', MarkerArray, queue_size=10)
        self.tf_pub = tf2_ros.TransformBroadcaster()

        tf2_ros.TransformListener(tf_buff)

        self.name = 'Unnamed course'
        self.challenges = []
        self.loadConfig(config_file)

    def loadConfig(self, config_file):
        """
        Loads data from the JSON configuration file provided.
        Note that any field objects not present in the file will be silently ignored, and neither
        their visualization nor their tf frames will be published.

        :param (string) config_file: The filename of the config file
        """
        with open(config_file) as f:
            data = json.load(f)

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

    def run(self, rate=5):
        """
        Runs the node, continually republishing the visualization markers and transform frames.
        :param (int) rate: Base publishing frequency, in hz
        :return:
        """

        print 'Running'

        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.marker_pub.publish(MarkerArray(self.as_markers()))
            self.tf_pub.sendTransform(self.as_transforms())

            r.sleep()


if __name__ == '__main__':
    TFHandler().run()
