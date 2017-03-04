#!/usr/bin/env python

import os
import subprocess
from time import sleep

ARDUINO_INSTALL_LOCATION = '/usr/local/arduino-1.8.1'
CODE_LOCATION = '/home/eric/Arduino/fastblink/fastblink.ino'
PORT = '/dev/ttyUSB0'
arch = None


def load_code():
    """
    Installs code from CODE_LOCATION to the PATH of the arduino
    :return: Whether the operation successfully installed the code
    :rtype: bool
    """

    if not os.path.exists(PORT):
        print "Requested arduino port {} does not exist"
        return False

    print "Beginning installation of {file} on {port}".format(file=CODE_LOCATION.split('/')[-1], port=PORT)

    if arch:
        boardstr = '--board {}'.format(arch)
    else:
        boardstr = ''

    command = """
    {install}/arduino --upload --port {port} {board} {code}
    """.format(port=PORT, install=ARDUINO_INSTALL_LOCATION, board=boardstr, code=CODE_LOCATION)

    p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    result = p.wait()

    if result == 0:
        print "Install complete"

        return True

    else:
        print "Install failed with error code {}\n".format(result)
        print p.communicate()[1]
        return False


def run_rosserial(port=PORT):
    if not os.path.exists(port):
        return None

    command = 'export ROS_NAMESPACE={ns};\
    rosrun rosserial_python serial_node.py {port}'.format(ns=port.split('/')[-1], port=port)

    p = subprocess.Popen(command, shell=True)
    return p


def maintain_rosserials(paths):
    # activeShells['roscore'] = subprocess.Popen('export ROS_IP=$(hostname --all-ip-addresses);roscore;', shell=True)
    try:
        while True:
            refreshShells(paths)
            sleep(5)
    except Exception, e:
        raise e
    finally:
        killAllShells()

activeShells = {}


def refreshShells(paths):
    for k in activeShells.keys():
        if activeShells[k].poll() is not None:
            print 'ROSserial process for {} has died'.format(k)
            # The shell has finished
            del activeShells[k]

    for path in paths:

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


if __name__ == '__main__':
    load_code()
