#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from math import atan2, degrees
rospy.init_node('imu_test')

def imu_callback(data):
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    w = data.orientation.w
    calc_heading_angle(w, x, y, z)


def calc_heading_angle(w, x, y, z):
    yaw_rad = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))
    yaw_deg = degrees(yaw_rad)
    print(yaw_deg)
    return yaw_deg    
    
imu_sub = rospy.Subscriber('/imu_data',Imu, imu_callback)
    
rospy.spin()    
