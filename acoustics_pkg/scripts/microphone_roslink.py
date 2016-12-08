#! /usr/bin/python

# import rospy
# import audio_common_msgs

# rospy.init_node('mic_input', anonymous=True)


import roslaunch

package = 'acoustics_pkg'
executable = 'mic_input'
node = roslaunch.core.Node(package, executable)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
print process.is_alive()
process.stop()