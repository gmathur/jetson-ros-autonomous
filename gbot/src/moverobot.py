import rospy
from geometry_msgs.msg import Twist

def callback(msg):
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    
def listener():
    rospy.init_node('gbot')
    rospy.Subscriber("cmd_vel", Twist, callback)
    while not rospy.is_shutdown(): 
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

def main():
    listener()

if __name__ == "__main__":
    main()
