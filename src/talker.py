#!/usr/bin/env python
# -*- coding: utf-8 -*-
from tf import TransformListener
from time import sleep
import rospy

import sys
import numpy as np
import copy
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
import tf

from math import pi
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
     except rospy.ROSInterruptException:
        pass

