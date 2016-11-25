#!/usr/bin/env python

# Author : Akshay Raj Dayal
# Date :19/04/2106
# This code adds a timestanp value to cmd_vel messages using simulated clock time . Its useful for Huksy cmd_vel
# which is processed without a timestamp by default 

import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math

i=1

#pub=rospy.Publisher('cmd_vel_stamped',TwistStamped,queue_size=10)
#pub=rospy.Publisher('/stereo_odometer/odometry/driftless',Odometry,queue_size=10)
pub=rospy.Publisher('/odometry/filtered/angles',Odometry,queue_size=10) 

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    twist=TwistStamped()
    twist.twist.linear=msg.linear
    twist.twist.angular=msg.angular
    twist.header.stamp=rospy.get_rostime()
    twist.header.seq=i+1
    pub.publish(twist)
    # Do velocity processing here:



    # Use the kinematics of your robot to map linear and angular velocities into motor commands


    # Then set your wheel speeds (using wheel_left and wheel_right as examples)

def viso_callback(msg):
    odom=Odometry()
    #imu=msg
    odom=msg
    #rospy.loginfo("Received an Imu message message!")
    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    #imu=Imu()
    
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    odom.pose.pose.orientation.x = roll*180/math.pi
    odom.pose.pose.orientation.y = pitch*180/math.pi
    odom.pose.pose.orientation.z = yaw*180/math.pi
    rospy.loginfo(yaw*180/math.pi)
    pub.publish(odom)

def listener():

    #rospy.init_node('cmd_vel_listener')
    #rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist, callback)
    #pub=rospy.Publisher('cmd_vel_stamped',TwistStamped,queue_size=10)
    rospy.init_node('driftless')
    rospy.Subscriber("/stereo_odometer/odometry", Odometry, viso_callback)
    #rospy.Subscriber("/imu/data", Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
