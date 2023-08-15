#!/usr/bin/env python3

import math
from math import sin, cos, pi
import serial
import rospy
import tf
import time
import re
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu


def talker():
    rospy.init_node('imu_odom_publisher')
    rospy.loginfo("Starting imu sensor setup...")

    # get parameter
    imu_port = rospy.get_param('~port', '/dev/ttyUSB0') #"""dmesg grep usb : CP210X"""
    baud = rospy.get_param('~baud', 115200)


    # initialize IMU w/ serial commands
    imu_data = Imu()
    odom_data = Odometry()
    # note: both imu_data and odom_data implies CW increase of yaw

    ser = serial.Serial(imu_port, baud)
    ser.write(b'<sof2>') # set quaternion output
    rospy.loginfo("imu setup: quaternion ON")
    ser.readline()
    ser.write(b'<sog1>') # set gyro (angular velocity) data output
    rospy.loginfo("imu setup: angular velocity ON")
    ser.readline()
    ser.write(b'<soa4>') # set global(5)/local(4) linear velocity data output
    rospy.loginfo("imu setup: linear velocity ON")
    ser.readline()
    ser.write(b'<sem1>') # set magnetometer on
    rospy.loginfo("imu setup: magnetometer ON")
    ser.readline()
    ser.write(b'<sod0>') # set local distance(calculated by intergration of velocity) data output 
    rospy.loginfo("imu setup: distance OFF")
    ser.readline()
    ser.write(b'<sor10>') # set output rate to 100ms = 10hz
    rospy.loginfo("imu setup: 10hz output rate")
    ser.readline()
    # ser.write('<lpf20>') # set low pass filter (cutoff f=10hz)
    # rospy.loginfo("imu setup: LPF (f=10hz) ON")
    # ser.readline()

    imu_pub = rospy.Publisher("imu_data", Imu, queue_size=5)
    odom_pub = rospy.Publisher("/odometry/imu", Odometry, queue_size=5)
    odom_broadcaster = tf.TransformBroadcaster()
    stab_broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(30) # 10hz
    x = 0
    y = 0
    th = 0
    prev_str = ser.readline() # when prev_str is bad input -> :(
    last_time = rospy.Time(0)

    rospy.loginfo("start ebimu node")
    while not rospy.is_shutdown():
        ser.reset_input_buffer()

        str_temp = ser.readline().decode()
        comma_cnt = len([m.start() for m in re.finditer(',', str_temp)])
        if (comma_cnt != 9):
            str_list = prev_str
            rospy.logwarn("invalid message from imu. ignore data for once")
        else:
            str_list = str_temp
            prev_str = str_temp
        
        str_list = str_list.split(',')
        str_list[0] = str_list[0].split('*')[1]
        imu_data.header.stamp = rospy.Time.now()
        imu_data.header.frame_id = "base_link"
        # note: EBIMU returns quaternion vector in order of [z, y, x, w]
        # covariance data estimated from ebimu datasheet
        imu_data.orientation.z = float(str_list[0])
        imu_data.orientation.y = float(str_list[1])
        imu_data.orientation.x = float(str_list[2])
        imu_data.orientation.w = float(str_list[3])
        imu_data.orientation_covariance = [0.0007, 0, 0, 0, 0.0007, 0, 0, 0, 0.0007]
        imu_data.linear_acceleration.x = float(str_list[7])
        imu_data.linear_acceleration.y = float(str_list[8])
        imu_data.linear_acceleration.z = float(str_list[9])
        imu_data.linear_acceleration_covariance = [0.005, 0, 0, 0, 0.005, 0, 0, 0, 0.005]
        imu_data.angular_velocity.x = math.radians(float(str_list[4]))
        imu_data.angular_velocity.y = math.radians(float(str_list[5]))
        imu_data.angular_velocity.z = math.radians(float(str_list[6]))
        imu_data.angular_velocity_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
        imu_pub.publish(imu_data)

        odom_quat = (float(str_list[2]),float(str_list[1]),float(str_list[0]),float(str_list[3]))
        euler = tf.transformations.euler_from_quaternion(odom_quat)

        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        th = euler[2]
        vx = float(str_list[7])
        vy = float(str_list[8])
        dx = (vx * cos(th) - vy * sin(th)) * dt
        dy = (vx * sin(th) + vy * cos(th)) * dt
        # dx = vx * dt
        # dy = vy * dt
        x += dx
        y += dy

        odom_quat_reverse = tf.transformations.quaternion_from_euler(euler[0], -euler[1], -euler[2])
        odom_quat_reverse_2d = tf.transformations.quaternion_from_euler(0, 0, -euler[2])


        odom_data.pose.pose.orientation.z = float(str_list[0])
        odom_data.pose.pose.orientation.y = float(str_list[1])
        odom_data.pose.pose.orientation.x = float(str_list[2])
        odom_data.pose.pose.orientation.w = float(str_list[3])
        odom_data.pose.pose.position.x = 0
        odom_data.pose.pose.position.y = 0
        odom_data.pose.pose.position.z = 0
        odom_data.twist.twist.linear.x = 0
        odom_data.twist.twist.linear.y = 0
        odom_data.twist.twist.linear.z = 0
        odom_data.twist.twist.angular.x = 0
        odom_data.twist.twist.angular.y = 0
        odom_data.twist.twist.angular.z = 0

        odom_quat_reverse = tf.transformations.quaternion_from_euler(-euler[0], euler[1], euler[2])
        odom_quat_reverse_2 = tf.transformations.quaternion_from_euler(euler[0], -euler[1], 0 )
        odom_quat_reverse_2d = tf.transformations.quaternion_from_euler(0, 0, -euler[2])
        odom_broadcaster.sendTransform(
            (0, 0, 0),
            odom_quat_reverse,
            rospy.Time.now(),
            "base_footprint","base_link"
            
        )

        odom_data.header.stamp = rospy.Time.now()
        odom_data.header.frame_id = "base_link"
        odom_data.child_frame_id = "base_footprint"

        odom_pub.publish(odom_data)

        prev_str = str_list
        last_time = current_time
        # end_time = time.time()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

