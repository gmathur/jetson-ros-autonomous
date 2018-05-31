#!/usr/bin/env python
import rospy
import roslib
from math import sin, cos, pi, asin
import tf
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

class OdomFromTf:

    def __init__(self):
        rospy.init_node("odom_from_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        self.world_frame_id = rospy.get_param('~world_frame_id', 'world') # the name of the world reference frame
 
        # subscriptions
        self.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.listener = tf.TransformListener()
        self.last_twist = [0, 0, 0]

        self.listener.waitForTransform("/%s" % (self.base_frame_id), "/%s" % (self.world_frame_id), 
                rospy.Time(), rospy.Duration(4.0))
        self.then = rospy.Time.now()

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def update(self):
        now = rospy.Time.now()
        elapsed = self.then - now

        (trans, rot) = self.listener.lookupTransform("/%s" % (self.base_frame_id), "/%s" % (self.world_frame_id), rospy.Time(0))
        
        dt = elapsed.to_sec()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = trans[0]
        odom.pose.pose.position.y = trans[1]
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation.x = rot[0]
        odom.pose.pose.orientation.y = rot[1]
        odom.pose.pose.orientation.z = rot[2]
        odom.pose.pose.orientation.w = rot[3]
        odom.pose.covariance = self.covariance
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = trans[0] - self.last_twist[0]
        odom.twist.twist.linear.y = trans[1] - self.last_twist[1]
        odom.twist.twist.angular.z = trans[2] - self.last_twist[2]
        odom.twist.covariance = self.covariance
        self.odomPub.publish(odom)
        self.then = now

        self.last_twist = trans
            

if __name__ == '__main__':
    try:
        odomFromTf = OdomFromTf()
        odomFromTf.spin()

    except rospy.ROSInterruptException:
        pass
