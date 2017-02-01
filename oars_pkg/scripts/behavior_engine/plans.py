"""
This file contains the database of plans the robot can execute.
It is intended to be modified frequently.
"""
from tasks import *

plans = [
    Plan('thefirstplan', [
        SampleTask(),
        DelayTask(2),
        GPSNavigationTask('Navigate home', destination='home'),
        RCTask()
    ]),
    Plan('fastplan', [
        DelayTask(2),
    ]),

]
