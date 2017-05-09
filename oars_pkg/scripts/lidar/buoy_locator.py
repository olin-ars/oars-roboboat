""" Handles processing LIDAR data (published via ROS) in search of circles. """
import numpy as np
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
    MAX_BUOY_RAD = 0.25 #0.3   # Maximum buoy diameter (m)

    def __init__(self, node_name=DEFAULT_ROS_NODE_NAME):
        rospy.init_node(node_name)
        # Subscribe to the LIDAR scan topic
        rospy.Subscriber(self.LIDAR_SCAN_TOPIC, LaserScan, self.process_scan)
        self.circle_finder = CircleFinder(self.MIN_BUOY_RAD, self.MAX_BUOY_RAD)
        # Publish to the '/buoys' topic
        self.pub = rospy.Publisher('buoys', msg.MarkerArray, queue_size=1)
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
        for c in circles:
            m = msg.Marker()
            h = std_msg.Header()
            h.stamp = timestamp
            h.frame_id = scan_msg.header.frame_id
            m.header = h
            m.type = m.SPHERE
            m.action = m.ADD
            m.scale.x = c.radius
            m.scale.y = c.radius
            m.scale.z = c.radius
            m.color = ColorRGBA(r=1, g=1, b=0, a=1)
            m.pose.position.x = c.center_x
            m.pose.position.y = c.center_y
            m.pose.position.z = 0

            self.marker_array.markers.append(m)
            print('Found circle at ({:0.3},{:0.3}) with radius {:0.3f}m...'.format(c.center_x, c.center_y, c.radius))

        self.pub.publish(self.marker_array)



    def process_scan_in_cartesian(self, point_cloud):
        """ Processes a LIDAR scan in order to find circles that mark buoys.
            :param point_cloud: a Cartesian point cloud of all the LIDAR points - sensor_msgs/PointCloud
        """


class CircleFinder:
    """ Processes a PointCloud in order to locate circular objects. """

    def __init__(self, min_rad, max_rad):
        """ Initializes a CircleFinder for finding circles in a PointCloud. It will only find circles with a radius in
            the specified range.
            :param min_rad: the minimum radius of the circles (m)
            :param max_rad: the maximum radius of the circles (m)
        """
        self.min_rad = min_rad
        self.max_rad = max_rad

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
            # print('Point at ({},{})'.format(p[0], p[1]))
            x[i] = p[0]
            y[i] = p[1]

        # Iterate through the points, 3 at a time, and try to fit a circle to each set of points
        circles = list()
        points_to_look_at = 6
        num_extra_points = num_points % points_to_look_at
        for i in range(0, num_points-num_extra_points, points_to_look_at):
            circle = self._fit_circle(x[i:i+points_to_look_at-1], y[i:i+points_to_look_at-1])
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
        r = math.sqrt(xc**2 + yc**2 - w[3])
        return Circle(xc, yc, r) if self.min_rad < r < self.max_rad else None


class Circle:

    def __init__(self, center_x, center_y, radius):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius

if __name__ == '__main__':
    BuoyLocator().run()
