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
from std_msgs.msg import String,Vector3D

rho = 1000 #kg.m-3
def archimede_sphere(r,h,rho):
    """h est la hauteur de sphère immergée
       r est le rayon de la sphère
       pho la masse volumique du liquide"""
    g = 9.81 #m.s-1
    V =  pi*h**2*(3*R-2*R)/3
    fA = -rho*V*g
    return fA

def Traction(rho,n,D,Ct):
	""" rho masse volumique du liquide 
        n vitesse de rotation de l'hélice en tr/s
        D diamètre de l'hélice en m
        Ct coefficient de traction à déterminer """
    Ft = rho*Ct*n**2*D**4
    return Ft

def Traine(n):
    """ n vitesse de rotation de l'hélice en tr/s
        alpha coefficient à déterminer"""
    alpha = 1000
    return alpha*n
def frottement(v)
    """v la vitesse du bateau"""
    beta = 1000
    return -v**2*beta

	
