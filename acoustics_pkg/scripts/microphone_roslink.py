#! /usr/bin/python

import rospy
import roslaunch

class ROShandler():
    """docstring for ROShandler"""
    def __init__(self, model):
		rospy.init_node('mic_input', anonymous=True)
		rospy.on_shutdown(self.shutdown)

		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		launch = roslaunch.parent.ROSLaunchParent(uuid, ["/opt/ros/indigo/share/audio_capture/launch/capture.launch"])

		launch.start()
		launch.shutdown()
