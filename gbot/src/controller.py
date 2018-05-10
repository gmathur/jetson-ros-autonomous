#!/usr/bin/env python
import curses
import rospy
from std_msgs.msg import String

def main(win):
    camera_pub = rospy.Publisher("camera_commands", String, queue_size=10)

    try:
        win.nodelay(True)
        key=""
        win.clear()                
        win.addstr("Move camera using W A S D - Q to quit")
        while True:          
            try:                 
                key = win.getkey()
                key = key.upper()
                if key == 'Q':
                    break
                
                if key in ['W', 'A', 'S', 'D']:
                    msg = String()
                    msg.data = key
                    camera_pub.publish(msg)

            except Exception as e:
               # No input   
               pass  
    finally:
        pass

curses.wrapper(main)
