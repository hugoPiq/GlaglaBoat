import tf
import time
import rospy
import sys
import numpy as np
import copy

import std_msgs
import sensor_msgs
import std_msgs

HEADING = 0


def control(heading_bar = 0, v_bar = 120):
    global HEADING
    rospy.init_node('controler', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', std_msgs.msg.Float64MultiArray, queue_size=10)
    rospy.Subscriber('/MagField', sensor_msgs.msg.MagneticField, callback)
    rate = rospy.Rate(10)  # 10hz
    
    K = 1/np.pi
    
    while not rospy.is_shutdown():
        m = std_msgs.msg.Float64MultiArray()
        
        e = sawtooth(HEADING - heading_bar)
        print(HEADING)
        
        u1 = int(0.5*v_bar*(1 + K*e))
        u2 = int(0.5*v_bar*(1 - K*e))
        
        m.data = [float(u1), float(u2)]
        
        pub.publish(m)
        rate.sleep()


def sawtooth(x):
    return (x+2*np.pi) % (2*np.pi)-np.pi


def callback(data):
    global HEADING
    mag_field = data.magnetic_field
    HEADING = np.arctan2(mag_field.y, mag_field.x)


if __name__ == "__main__":
    try:
        control()
    except rospy.ROSInterruptException:
        pass

