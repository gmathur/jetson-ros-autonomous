#!/usr/bin/env python
import rospy
from dnn_detect.msg import DetectedObjectArray, DetectedObject

class DetectPeople:
    def __init__(self):
        rospy.Subscriber("dnn_objects", DetectedObjectArray, self.process_objects, queue_size=1)

    def process_objects(self, data):
        for person in data.objects:
            rospy.loginfo("x min %d x max %d y min %d y max %d", person.x_min, person.x_max, person.y_min, person.y_max)

if __name__ == "__main__":
    rospy.init_node('process_dnn_objects')
 
    processor = DetectPeople()
    rospy.spin()
