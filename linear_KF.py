#!/usr/bin/env python

# Author : Abhishek Raj Dutta
# Date :22/11/2106
# This code merges yaw from stereo odometry and an IMU sensor to give a Kalman Filtered estimate

import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
import numpy as np

pub=rospy.Publisher('/odometry/filtered/angles',Odometry,queue_size=10)
odom=Odometry()
imu=Imu()
yawViso=0.0
vyawViso=0.0
yawImu=0.0
yawImu1=0.0
yaw=0.0
i=0
vYawCov=0.09
ImuYawCov=0.15707963267948966
yawCov=10
cmd = np.array ((0.0,0.0))
#frequency = 2 Hz
f=5.0
t=1/f


def viso_callback(msg):    
    global odom
    odom=msg
    quaternion1 = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion1)
    roll = euler[0]
    pitch = euler[1]
    global yawViso,vyawViso
    yawViso = euler[2]
    vyawViso=msg.twist.twist.angular.z;
    #odom.pose.pose.orientation.x = roll*180/math.pi
    #odom.pose.pose.orientation.y = pitch*180/math.pi
    #odom.pose.pose.orientation.z = yaw*180/math.pi
    

def imu_callback(msg):
    global imu
    imu=msg
    quaternion = (
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z,
    msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    global yawImu
    yawImu = euler[2]
    #imu.orientation.x = roll*180/math.pi
    #imu.orientation.y = pitch*180/math.pi
    #imu.orientation.z = yaw*180/math.pi

def getCmd(msg):
    global cmd
    cmd[0] = msg.twist.linear.x
    cmd[1] = msg.twist.angular.z
    
    

def loop(event):
    print 'Timer called at ' + str(event.current_real)
    global i,yawImu,yawImu1,vyawViso,vYawCov,ImuYawCov,yaw,yawCov
    if i==0:
        yawImu1=yawImu
        i=1
    
    #linear KF
    predYaw=yaw+vyawViso*t
    predCov=yawCov + vYawCov
    inn=(yawImu-yawImu1)-predYaw
    innCov=predCov+ImuYawCov
    K=predCov*(1/innCov)
    yaw=predYaw+K*inn
    yawCov=(1-K)*predCov
    

    rospy.loginfo(yaw*180/math.pi)
    rospy.loginfo(yawViso*180/math.pi)
    rospy.loginfo(vyawViso*180/math.pi)
    rospy.loginfo((yawImu-yawImu1)*180/math.pi)
    print cmd
    pub.publish(odom)



def ekf():
	rospy.init_node('ekf_node')
	rospy.Subscriber("/stereo_odometer/odometry", Odometry, viso_callback)
	rospy.Subscriber("/cmd_vel_stamped", TwistStamped, getCmd)
	rospy.Subscriber("/imu/data", Imu, imu_callback)
	rospy.Timer(rospy.Duration(t), loop)
	rospy.spin()

if __name__ == '__main__':
    ekf()





