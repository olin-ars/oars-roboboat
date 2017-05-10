"""
This file contains the database of plans the robot can execute.
It is intended to be modified frequently.
"""
from tasks import *
import objects
import smach

firstsm = smach.StateMachine(outcomes=['completed'])

with firstsm:
    # add states to the state machine
    smach.StateMachine.add('SAMPLE', SampleTask(),
                           transitions={'done': 'DELAY'})

    smach.StateMachine.add('DELAY', DelayTask(2),
                           transitions={'done': 'GPSNAV'})

    smach.StateMachine.add('GPSNAV', GPSNavigationTask('Navigate home', destination='home'),
                           transitions={'done': 'RC'})

    smach.StateMachine.add('RC', RCTask(),
                           transitions={'done': 'completed'})

fastsm = smach.StateMachine(outcomes=['completed'])

with fastsm:
    smach.StateMachine.add('DELAY', DelayTask(2),
                           transitions={'done': 'completed'})

navigation_challenge = smach.StateMachine(outcomes=['completed', 'failed'])

with navigation_challenge:
    smach.StateMachine.add('GO_ENTRANCE', WaypointNavigationTask(('navigation_entrance', -2., 0.)),
                           transitions={'done': 'DELAY_1'})
    smach.StateMachine.add('DELAY_1', DelayTask(5),
                           transitions={'done': 'GO_THROUGH_1'})
    smach.StateMachine.add('GO_THROUGH_1', WaypointNavigationTask(('navigation_exit', -2., 0.)),
                           transitions={'done': 'DELAY_2'})
    smach.StateMachine.add('DELAY_2', DelayTask(5),
                           transitions={'done': 'GO_ENTRANCE_2'})
    smach.StateMachine.add('GO_ENTRANCE_2', WaypointNavigationTask(('navigation_exit', -2., 0.)),
                           transitions={'done': 'DELAY_3'})
    smach.StateMachine.add('DELAY_3', DelayTask(5),
                           transitions={'done': 'GO_THROUGH_2'})
    smach.StateMachine.add('GO_THROUGH_2', WaypointNavigationTask(('navigation_exit', -2., 0.)),
                           transitions={'done': 'completed'})

test1 = smach.StateMachine(outcomes=['completed', 'failed'])

with test1:
    smach.StateMachine.add('DELAY_START', DelayTask(10),
                           transitions={'done': ''})
    smach.StateMachine.add('NAVIGATION', navigation_challenge)

plans = {
    'firstplan': firstsm,
    'fastplan': fastsm,
    'navigation': navigation_challenge,
    'test1': test1
}
