#!/usr/bin/python

import rospy, time
from std_msgs.msg import Bool

class Main(object):
	"""docstring for Main"""
	def __init__(self,):
		super(Main, self).__init__()
		self.startSub = rospy.Subscriber('/test_task_active', Bool, self.onEnable)
		self.donePub = rospy.Publisher('/test_task_done', Bool, queue_size=10)
		self.running = False

	def onEnable(self, msg):
		if msg.data == True:
			self.running = True
		else:
			self.running = False

	def run(self):
		rospy.init_node('sample_task_node')
		rate = rospy.Rate(5)  # 5hz refresh rate

		i = 0
		while not rospy.is_shutdown():
			if self.running:
				i += 1
				print("I'm running! {}".format(i))

				if i > 30:
					self.running = False
					self.donePub.publish(Bool(True))
					i = 0


			rate.sleep()


		
if __name__ == '__main__':
	Main().run()