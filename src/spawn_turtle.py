#! /usr/bin/env python2
import rospy
import rosservice
import random
from math import pi
import time

# random values
x=random.random()*10
y=random.random()*10
theta=random.random()*pi

# # take input from user
# x = input("Enter X value: ")
# y = input("Enter y value: ")
# theta = input("Enter angle in degree: ")*(pi/180)

rospy.wait_for_service('/spawn')
#Waiting for 10 seconds
time.sleep(10)
#calling service
rosservice.call_service("/spawn",[x, y, theta, ""])
