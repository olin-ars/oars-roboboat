import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, Pose, Vector3

rospy.init_node('probably_still')

pub = rospy.Publisher('/probably_still', Odometry, queue_size=10)

r = rospy.Rate(10)

xy_variance = 1.0
z_variance = 0.1

msg = Odometry(
    header=Header(frame_id='map'),
    pose=PoseWithCovariance(pose=Pose(position=Vector3(x=10)),
                            covariance=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
)

while not rospy.is_shutdown():
    r.sleep()
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
