#!/usr/bin/env python
# import rospy
# from std_msgs.msg import Int16

import subprocess
import os.path
from time import sleep

activeShells = {}


def getIP():
    return subprocess.check_output(['hostname', '--all-ip-addresses']).strip()


def run_rosserial(port='/dev/ttyACM0'):
    if (not os.path.exists(port)):
        return None;

    command = 'export ROS_NAMESPACE={ns};\
    rosrun rosserial_python serial_node.py {port}'.format(ns=port.split('/')[-1], port=port)

    p = subprocess.Popen(command, shell=True)
    return p


def refreshShells():
    for k in activeShells.keys():
        if activeShells[k].poll() is not None:
            print 'ROSserial process for {} has died'.format(k)
            # The shell has finished
            del activeShells[k]

    for i in range(4):
        path = '/dev/ttyACM{}'.format(i)

        if path in activeShells:
            print 'ROSserial running on {}'.format(path)
            continue
        else:
            p = run_rosserial(path)
            if p:
                activeShells[path] = p
                print 'Started ROSserial for {}'.format(path)
            else:
                print 'Unable to start ROSserial for {}'.format(path)
    print ''


def killAllShells():
    print '\nShutting down all active shells'
    for k in activeShells:
        activeShells[k].terminate()
    sleep(2)


def main():
    # activeShells['roscore'] = subprocess.Popen('export ROS_IP=$(hostname --all-ip-addresses);roscore;', shell=True)
    try:
        while True:
            refreshShells()
            sleep(3)
    except Exception, e:
        raise e
    finally:
        killAllShells()


if __name__ == '__main__':
    main()
