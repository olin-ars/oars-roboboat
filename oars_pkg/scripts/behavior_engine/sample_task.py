#!/usr/bin/python

import rospy, time
from std_msgs.msg import Bool

rospy.init_node('sample_task_node')

class Main(object):
    """docstring for Main"""

    def __init__(self, ):
        super(Main, self).__init__()
        self.startSub = rospy.Subscriber('/test_task_active', Bool, self.onEnable)
        self.donePub = rospy.Publisher('/test_task_done', Bool, queue_size=10)

        self.running = False
        self.i = 0

    def onEnable(self, msg):
        if msg.data == True:
            self.running = True
            self.i = 0
        else:
            self.running = False

    def finishTask(self):
        self.donePub.publish(Bool(True))

    def run(self):
        r = rospy.Rate(5)  # 5hz refresh rate

        while not rospy.is_shutdown():
            if self.running:
                self.i += 1
                print("I'm running! {}".format(self.i))

                if self.i >= 20:
                    self.finishTask()

            r.sleep()


if __name__ == '__main__':
    Main().run()
