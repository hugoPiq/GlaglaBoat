# import tf
import time
import rospy
import sys
import numpy as np
import copy

import std_msgs
import sensor_msgs

HEADING = 0


def control(heading_bar=np.pi/4, v_bar=200):
    global HEADING
    rospy.init_node('controler', anonymous=True)
    pub = rospy.Publisher(
        '/cmd_vel', std_msgs.msg.Float64MultiArray, queue_size=10)
    # rospy.Subscriber('/MagField', sensor_msgs.msg.MagneticField, callback)
    rospy.Subscriber(
        '/Compass', std_msgs.msg.Float64MultiArray, compass_callback)
    rate = rospy.Rate(10)  # 10hz

    K = 1/np.pi

    while not rospy.is_shutdown():
        m = std_msgs.msg.Float64MultiArray()

        e = abs(sawtooth(HEADING - heading_bar))
        print("HEADING : ", HEADING)
        print("e : ", e)

        if e <= 0.005:
            e = 0

        u1 = int(0.5*v_bar*(1 - K*e))
        u2 = int(0.5*v_bar*(1 + K*e))

        m.data = [float(u1), float(u2)]
        print(float(u1))
        print(float(u2))

        pub.publish(m)
        rate.sleep()


def sawtooth(x):
    return (x+np.pi) % (2*np.pi)-np.pi


def callback(data):
    global HEADING
    mag_field = data.magnetic_field
    HEADING = np.arctan2(mag_field.z, mag_field.y)


def compass_callback(msg):
    global HEADING
    HEADING = np.arctan2(msg.data[1], msg.data[0])


if __name__ == "__main__":
    try:
        control()
    except rospy.ROSInterruptException:
        pass
