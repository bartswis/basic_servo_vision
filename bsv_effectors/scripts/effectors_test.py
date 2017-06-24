#!/usr/bin/env python

import os
import sys
import termios
import fcntl
import rospy
from std_msgs.msg import String

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
    pub = rospy.Publisher('bsv/system_to_effector_cmd', String, queue_size=1)
    rospy.init_node('effectors_test', anonymous=True)

    while not rospy.is_shutdown():

        char = getch()
        char = ord(char)
            
        if char == 119:  # w
            pub.publish('0;10')
            print 'up'

        elif char == 115:  # s
            pub.publish('0;-10')
            print 'down'

        elif char == 97:  # a
            pub.publish('-10;0')
            print 'left'

        elif char == 100:  # d
            pub.publish('10;0')
            print 'right'

#MAIN
if __name__ == '__main__':

    print 'Start'

    try:
        effectors_tester()
    except rospy.ROSInterruptException:
        pass


