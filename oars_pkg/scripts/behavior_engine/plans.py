"""
This file contains the database of plans the robot can execute.
It is intended to be modified frequently.
"""
from tasks import *
import smach

firstplan = smach.StateMachine(outcomes=['completed'])

with firstplan:
	#add states to the state machine
	smach.StateMachine.add('SAMPLE', SampleTask(),
							transitions={'done': 'DELAY'})

	smach.StateMachine.add('DELAY', DelayTask(2),
							transitions={'done': 'GPSNAV'})

	smach.StateMachine.add('GPSNAV', GPSNavigationTask('Navigate home', destination='home'),
							transitions={'done': 'RC'})

	smach.StateMachine.add('RC', RCTask(),
							transitions={'done': 'completed'})

fastplan = smach.StateMachine(outcomes = ['completed'])

with fastplan:

	smach.StateMachine.add('DELAY', DelayTask(2),
							transitions={'done': 'completed'})

plans = [
	('thefirstplan', firstplan)
]