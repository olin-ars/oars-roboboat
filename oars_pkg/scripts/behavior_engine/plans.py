#!/usr/bin/python
from tasks import *

tasklist = [
TopicTask("Test task", "/test_task_active", "/test_task_done"),
RCTask()
]

activeTask = -1

def runNextTask():
	global activeTask
	activeTask += 1

	if activeTask < len(tasklist):
		tasklist[activeTask].start(runNextTask)

def test():
	rospy.init_node('task_test_node')
	# firsttask = TopicTask("Test task", "/test_task_active", "/test_task_done")
	runNextTask()

	rospy.spin()


if __name__ == '__main__':
	test()