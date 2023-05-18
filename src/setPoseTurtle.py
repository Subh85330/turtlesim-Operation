#! /usr/bin/env python2
import rospy
import rosservice
import random
from math import pi
import time

# random values
x=2
y=2
theta=0

rospy.wait_for_service('/s')
#Waiting for 10 seconds
time.sleep(10)
#calling service
rosservice.call_service("/s",[x, y, theta, ""])
