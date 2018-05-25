#!/usr/bin/env python
# -*- coding: utf-8 -*-

import curses
import rospy
import locale
from std_msgs.msg import String, Int16

def main(win):
    locale.setlocale(locale.LC_ALL,"")
    rospy.init_node('manual_controller')
    camera_pub = rospy.Publisher("camera_commands", String, queue_size=1)
    lmotor_pub = rospy.Publisher("lmotor", Int16, queue_size=1)
    rmotor_pub = rospy.Publisher("rmotor", Int16, queue_size=1)
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
                    manual_mode = Int16()
                    manual_mode.data = 1
                    manual_mode_pub.publish(manual_mode)

                    if key == "KEY_UP":
                        msg = Int16()
                        msg.data = 127
                        lmotor_pub.publish(msg)
                        rmotor_pub.publish(msg)
                    elif key == "KEY_DOWN":
                        msg = Int16()
                        msg.data = -127
                        lmotor_pub.publish(msg)
                        rmotor_pub.publish(msg)
                    elif key == "KEY_LEFT":
                        lmsg = Int16()
                        lmsg.data = -127
                        rmsg = Int16()
                        rmsg.data = 127

                        lmotor_pub.publish(lmsg)
                        rmotor_pub.publish(rmsg)
                    elif key == "KEY_RIGHT":
                        lmsg = Int16()
                        lmsg.data = 127
                        rmsg = Int16()
                        rmsg.data = -127

                        lmotor_pub.publish(lmsg)
                        rmotor_pub.publish(rmsg)


            except Exception as e:
               # No input   
               pass  
    finally:
        pass

curses.wrapper(main)
