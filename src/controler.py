import tf
import time
import rospy
import sys
import numpy as np
import copy

#import geometry_msgs.msg
#from geometry_msgs.msg import Pose, PoseStamped

#from math import pi
#from std_msgs.msg import String
import std_msgs
import sensor_msgs
import std_msgs

#from sensor_msgs.msg import MagneticField

HEADING = 0


def talker():
    pub = rospy.Publisher('/cmd_vel', std_msgs.msg.Float64MultiArray, queue_size=10)
    rospy.Subscriber('/MagField', sensor_msgs.msg.MagneticField, callback)
    rospy.init_node('controler', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    
    heading_bar = 0
    vmax = 80
    vmin = 40
    K = 1.15
    
    while not rospy.is_shutdown():
        m = std_msgs.msg.Float64MultiArray()
        
        e = sawtooth(HEADING - heading_bar)
        v = ((abs(e)*(vmax - vmin)) / np.pi) + vmin
        
        u1 = int(0.5*v*(1 + K*e))
        u2 = int(0.5*v*(1 - K*e))
        
        m.data = [float(u1), float(u2)]
        
        pub.publish(m)
        rate.sleep()


def sawtooth(x):
    return (x+2*np.pi) % (2*np.pi)-np.pi


def callback(data):
    mag_field = data.magnetic_field
    HEADING = np.arctan2(mag_field.y, mag_field.x)



if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

