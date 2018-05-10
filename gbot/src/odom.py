#!/usr/bin/env python
import rospy

class WheelEncoder:
    def __init__(self):
        self.lengthBetweenTwoWheels = 0.15
        self.left_previous_encoder = 0
        self.right_previous_encoder = 0
        self.radius = 0.065
        self.last_time = rospy.Time.now()
        self.ticks_per_rotation = 2626
        self.distancePerCount = (3.14159265 * 0.13) / self.ticks_per_rotation

    def process_encoder_message(self):
        current_time = rospy.Time.now()

        # extract the wheel velocities from the tick signals count
        deltaLeft = tick_x - self.left_previous_encoder;
        deltaRight = tick_y - self.right_previous_encoder;

        v_left = (deltaLeft * self.distancePerCount) / (current_time - last_time);
        v_right = (deltaRight * self.distancePerCount) / (current_time - last_time);

        vx = ((v_right + v_left) / 2)*10;
        vy = 0;
        vth = ((v_right - v_left)/ self.left_previous_encoder)*10;

        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th)) * dt;
        double delta_y = (vx * sin(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //Odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

         //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);
        _PreviousLeftEncoderCounts = tick_x;
        _PreviousRightEncoderCounts = tick_y;

        last_time = current_time;
