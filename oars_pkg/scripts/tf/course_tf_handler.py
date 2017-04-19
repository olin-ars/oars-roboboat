#!/usr/bin/env python

import rospy

import tf2_ros
import tf2_geometry_msgs

import json

class TFHandler(object):
    def __init__(self):
        super(TFHandler, self).__init__()

        rospy.init_node('course_tf_handler')

        self.configFile = rospy.get_param('~config')


    def loadConfig(self):
        with open(self.configFile) as file:
            data = json.load(file)
            print data

    def run(self):
        self.loadConfig()
        rospy.spin()


if __name__ == '__main__':
    TFHandler().run()

