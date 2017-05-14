#!/usr/bin/env python
""" Handles processing LIDAR data (published via ROS) in search of circles. """
import numpy as np
from geometry_msgs.msg import Point32
from scipy.sparse.linalg import lsqr
import math
import rospy
from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs import msg
from std_msgs import msg as std_msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud


# import sens


class BuoyLocator:
    DEFAULT_ROS_NODE_NAME = 'buoy_locator'
    LIDAR_SCAN_TOPIC = '/scan'
    MIN_BUOY_RAD = 0.075  # Minimum buoy diameter (m)
    MAX_BUOY_RAD = 0.3  # Maximum buoy diameter (m)
    CIRCLE_DETECT_THRESHOLD = 0.0001
    POINT_DIST_MIN = 0.8  # Don't look at points within this radius of the boat

    # More parameters for circle fitting in CircleFinder

    def __init__(self, node_name=DEFAULT_ROS_NODE_NAME):
        rospy.init_node(node_name)
        # Subscribe to the LIDAR scan topic
        rospy.Subscriber(self.LIDAR_SCAN_TOPIC, LaserScan, self.process_scan)
        self.circle_finder = CircleFinder(self.MIN_BUOY_RAD, self.MAX_BUOY_RAD, self.CIRCLE_DETECT_THRESHOLD)
        # Publish to the '/buoys' topic
        self.pub = rospy.Publisher('buoys', msg.MarkerArray, queue_size=1)
        self.point_pub = rospy.Publisher('/scan/circles', PointCloud, queue_size=1)
        self.marker_array = msg.MarkerArray()

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            while len(self.marker_array.markers) > 5:
                self.marker_array.markers.pop()
            r.sleep()

    def process_scan(self, scan_msg):
        """ Processes a LIDAR scan in order to find circles that mark buoys. 
            :param scan_msg: laser scan data from the LIDAR - sensor_msgs/LaserScan
        """
        cloud = LaserProjection().projectLaser(scan_msg)
        timestamp = scan_msg.header.stamp
        circles = self.circle_finder.process_point_cloud(cloud)

        points = PointCloud(header=scan_msg.header)

        for i, c in enumerate(circles):
            if self.POINT_DIST_MIN > math.sqrt(c.center_x ** 2 + c.center_y ** 2):
                continue

            m = msg.Marker()
            h = std_msg.Header()
            h.stamp = timestamp
            h.frame_id = scan_msg.header.frame_id
            m.header = h
            m.type = m.SPHERE
            m.action = m.ADD
            m.scale.x = c.radius * 2
            m.scale.y = c.radius * 2
            m.scale.z = c.radius * 2
            m.color = ColorRGBA(r=1, g=1, b=0, a=0.5)
            m.pose.position.x = c.center_x
            m.pose.position.y = c.center_y
            m.pose.position.z = 0

            m.id = i

            points.points.append(Point32(x=c.center_x, y=c.center_y, z=0))

            self.marker_array.markers.append(m)
            # print('Found circle at ({:0.3},{:0.3}) with radius {:0.3f}m...'.format(c.center_x, c.center_y, c.radius))

        self.pub.publish(self.marker_array)
        self.point_pub.publish(points)


class CircleFinder:
    """ Processes a PointCloud in order to locate circular objects. """

    def __init__(self, min_rad, max_rad, error_threshold):
        """ Initializes a CircleFinder for finding circles in a PointCloud. It will only find circles with a radius in
            the specified range.
            :param min_rad: the minimum radius of the circles (m)
            :param max_rad: the maximum radius of the circles (m)
            :param error_threshold: maximum value for the least squares error when finding circles
        """
        self.min_rad = min_rad
        self.max_rad = max_rad
        self.error_threshold = error_threshold

    def process_point_cloud(self, cloud):
        """ Looks for circles in the given point cloud.
            :param cloud: the Cartesian point cloud to look for circles - sensor_msgs/PointCloud
        """
        # Convert the PointCloud into numpy arrays
        gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
        num_points = cloud.width
        x = np.empty((num_points, 1), dtype=float)
        y = np.empty((num_points, 1), dtype=float)
        for i, p in enumerate(gen):
            x[i] = p[0]
            y[i] = p[1]

        # Iterate through the points, at a time, and try to fit a circle to each set of points
        circles = list()
        points_to_look_at = 10
        step = 2
        num_extra_points = num_points % step
        for i in range(0, num_points - num_extra_points, step):
            circle = self._fit_circle(x[i:i + points_to_look_at - 1], y[i:i + points_to_look_at - 1])
            if circle:
                circles.append(circle)
        return circles

    def _fit_circle(self, x, y):
        """ Fits a circle to the points using least squares.
        
            :param x: a column vector of x values - numpy.array
            :param y: a column vector of y values - numpy.array
            :return a Circle if one is found matching the radius constraints, None otherwise
            :rtype Circle
        """
        # Set up an overdetermined system of linear equations (A*w = b)
        o = np.ones((x.shape[0], 1), dtype=float)
        A = np.concatenate((x, y, o), 1)
        b = -np.power(x, 2) - np.power(y, 2)
        w = lsqr(A, b)
        # Convert from the least squares solution to the more familiar parameters of a circle.
        xc = -w[0][0] / 2.0
        yc = -w[0][1] / 2.0
        r = math.sqrt(xc ** 2 + yc ** 2 - w[0][2])
        ave_error = w[4] / len(x)
        return Circle(xc, yc, r) if (ave_error < self.error_threshold and self.min_rad < r < self.max_rad) else None


class Circle:
    def __init__(self, center_x, center_y, radius):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius


if __name__ == '__main__':
    BuoyLocator().run()
