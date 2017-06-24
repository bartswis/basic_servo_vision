#!/usr/bin/env python

import serial
import thread
import rospy
from time import sleep
from threading import Lock
from std_msgs.msg import String


h_pos = 90;
v_pos = 90;

m_h = Lock();
m_v = Lock();

def servo_comm( threadName):

    global h_pos
    global v_pos

    while not rospy.is_shutdown():
        m_v.acquire()
        s_v_pose = v_pos
        m_v.release()

        if s_v_pose < 60: s_v_pose = 60
        if s_v_pose > 120: s_v_pose = 120

        output = 'V'
        if s_v_pose < 100: output+='0'
        if s_v_pose < 10: output+='0'
        output += str(s_v_pose)
        ard.write(output);

        m_h.acquire()
        s_h_pose = h_pos
        m_h.release()

        if s_h_pose < 0: s_h_pose = 0
        if s_h_pose > 180: s_h_pose = 180

        output = 'H'
        if h_pos < 100: output+='0'
        if h_pos < 10: output+='0'
        output += str(h_pos)
        ard.write(output);

        sleep(0.1)
    ard.flush();

def move_rel(msg):

    global h_pos
    global v_pos

    buf = msg.data.split(';')

    m_h.acquire()
    h_pos+= int(buf[0])
    m_h.release()

    m_v.acquire()
    v_pos+= int(buf[1])
    m_v.release()
    

#MAIN
if __name__ == '__main__':

    print 'Start'

    try:
        port = '/dev/ttyUSB0'
        ard = serial.Serial(port,9600,timeout=0.1)
    except serial.serialutil.SerialException:
        try: 
            port = '/dev/ttyUSB1'
            ard = serial.Serial(port,9600,timeout=0.1)
        except serial.serialutil.SerialException:
            print "Error: unable find serial"
            sys.exit(0)

    try:
        thread.start_new_thread( servo_comm, ("Servos",) )
    except:
        print "Error: unable to start thread"
        sys.exit(0)

    rospy.init_node('effectors', anonymous=True)
    rospy.Subscriber("/bsv/system_to_effector_cmd", String, move_rel, queue_size=1)
rospy.spin() 
        

