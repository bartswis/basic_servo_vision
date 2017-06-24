#!/usr/bin/env python

import os
import sys
import termios
import fcntl
import serial
import thread
from time import sleep
from threading import Lock

global h_pos
global v_pos

m_h = Lock();
m_v = Lock();

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

def servo_comm( threadName):
    while running:
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

#MAIN
if __name__ == '__main__':

    print 'Start'

    h_pos = 90;
    v_pos = 90;

    try:
        port = '/dev/ttyUSB0'
        ard = serial.Serial(port,9600,timeout=0.1)
    except serial.serialutil.SerialException:
        try: 
            port = '/dev/ttyUSB1'
            ard = serial.Serial(port,9600,timeout=0.1)
        except serial.serialutil.SerialException:
            print "Error: unable to find serial"
            sys.exit(0)

    try:
        thread.start_new_thread( servo_comm, ("Servos",) )
    except:
        print "Error: unable to start thread"
        sys.exit(0)

    running = True
    print 'Running'

    while running:
        char = getch()
        char = ord(char)
            
        if char == 113:  # q
            running = False

        else:

            if char == 119:  # w
                m_v.acquire()
                v_pos+=10
                m_v.release()
                print 'up'

            elif char == 115:  # s
                m_v.acquire()
                v_pos-=10
                m_v.release()
                print 'down'

            elif char == 97:  # a
                m_h.acquire()
                h_pos-=10
                m_h.release()
                print 'left'

            elif char == 100:  # d
                m_h.acquire()
                h_pos+=10
                m_h.release()
                print 'right'

