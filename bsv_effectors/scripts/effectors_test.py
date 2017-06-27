#!/usr/bin/env python

import os
import sys
import termios
import fcntl
import rospy
from bsv_msg.msg import MoveAngle

def getch():
    fd = sys.stdin.fileno()

    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)

    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

    try:
        while 1:
            try:
                c = sys.stdin.read(1)
                break
            except IOError:
                pass
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
    return c

def effectors_tester():
    pub = rospy.Publisher('bsv_effectors/system_to_effector_cmd', MoveAngle, queue_size=1)
    rospy.init_node('effectors_test', anonymous=True)

    while not rospy.is_shutdown():

        char = getch()
        char = ord(char)

        msg = MoveAngle()
        msg.type = 'relative'
        msg.angle_hor = 0
        msg.angle_ver = 0
            
        if char == 119:  # w
            msg.angle_ver = 10
            print 'up'

        elif char == 115:  # s
            msg.angle_ver = -10
            print 'down'

        elif char == 97:  # a
            msg.angle_hor = -10
            print 'left'

        elif char == 100:  # d
            msg.angle_hor = 10
            print 'right'

        pub.publish(msg)

#MAIN
if __name__ == '__main__':

    print 'Start'

    try:
        effectors_tester()
    except rospy.ROSInterruptException:
        pass


