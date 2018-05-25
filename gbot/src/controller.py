#!/usr/bin/env python
# -*- coding: utf-8 -*-

import curses
import rospy
import locale
from std_msgs.msg import String, Int16, Float32

def main(win):
    locale.setlocale(locale.LC_ALL,"")
    rospy.init_node('manual_controller')
    camera_pub = rospy.Publisher("camera_commands", String, queue_size=1)
    lmotor_pub = rospy.Publisher("lwheel_vtarget", Float32, queue_size=1)
    rmotor_pub = rospy.Publisher("rwheel_vtarget", Float32, queue_size=1)
    manual_mode_pub = rospy.Publisher("manual_override", Int16, queue_size=1)

    camera_cmds = {
        "W": "U",
        "A": "L",
        "S": "D",
        "D": "R"
    }
    try:
        win.nodelay(True)
        key=""
        win.clear()
        
        row = int(curses.LINES/2)
        col = int((curses.COLS - 30)/2)
        win.addstr(row - 2, col, "Move robot using arrow keys")
        win.addstr(row, col+1, "Move camera using W A S D")
        win.addstr(row + 2, col + 9, "Q to quit")
        manual_override = False

        while True:          
            try:                 
                key = win.getkey()
                key = key.upper()
                if key == 'Q':
                    break
                
                if key in ['W', 'A', 'S', 'D']:
                    msg = String()
                    msg.data = camera_cmds[key]
                    camera_pub.publish(msg)

                if key in ['KEY_UP', 'KEY_DOWN', 'KEY_LEFT', 'KEY_RIGHT']:
                    if not manual_override:
                        manual_mode = Int16()
                        manual_mode.data = 1
                        manual_mode_pub.publish(manual_mode)
                        manual_override = True

                    if key == "KEY_UP":
                        msg = Float32()
                        msg.data = 0.30
                        lmotor_pub.publish(msg)
                        rmotor_pub.publish(msg)
                    elif key == "KEY_DOWN":
                        msg = Float32()
                        msg.data = -0.30
                        lmotor_pub.publish(msg)
                        rmotor_pub.publish(msg)
                    elif key == "KEY_LEFT":
                        lmsg = Float32()
                        lmsg.data = -0.30
                        rmsg = Float32()
                        rmsg.data = 0.30

                        lmotor_pub.publish(lmsg)
                        rmotor_pub.publish(rmsg)
                    elif key == "KEY_RIGHT":
                        lmsg = Float32()
                        lmsg.data = 0.30
                        rmsg = Float32()
                        rmsg.data = -0.30

                        lmotor_pub.publish(lmsg)
                        rmotor_pub.publish(rmsg)


            except Exception as e:
               # No input   
               pass  
    finally:
        pass

curses.wrapper(main)
