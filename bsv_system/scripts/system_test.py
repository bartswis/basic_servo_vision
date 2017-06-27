#!/usr/bin/env python

import os
import sys
import termios
import fcntl
import rospy
from bsv_msg.msg import MoveAngle
from geometry_msgs.msg import PoseStamped



def system_tester(msg):

        out_msg = MoveAngle()
        out_msg.type = 'relative'
        out_msg.angle_hor = msg.pose.position.x * 100
        out_msg.angle_ver = -msg.pose.position.y * 100
        pub.publish(out_msg)

#MAIN
if __name__ == '__main__':

    print 'Start'

    pub = rospy.Publisher('bsv_effectors/system_to_effector_cmd', MoveAngle, queue_size=1)
    rospy.init_node('system', anonymous=True)
    rospy.Subscriber("/bsv_camera_icube/vr_camera_icube/camera_icube_object_position", PoseStamped, system_tester, queue_size=1)
    rospy.spin() 


