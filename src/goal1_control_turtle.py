#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import  sqrt, atan2

class FlytbaseTurtle():
    def __init__(self):
        rospy.init_node('goal1_node')
        self.turtle_pose = Pose()
        self.turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        self.rate=rospy.Rate(10)
        self.turtle_pose_sub=rospy.Subscriber('/turtle1/pose',Pose,self.dataRecvCallback)
    
    def dataRecvCallback(self,turtle_pose_data_recv):
        self.turtle_pose=turtle_pose_data_recv

    def goToGoal(self,kp,kd,ki):
        x_goal = input("Enter Goal X: ")
        y_goal = input("Enter Goal Y: ")
        vel=Twist()
        
        e_sum = 0
        eclud_dist_prev = sqrt((self.turtle_pose.x-x_goal)**2 + (self.turtle_pose.y - y_goal)**2)

        
        steer_sum = 0
        steer_prev=(atan2(y_goal-self.turtle_pose.y, x_goal-self.turtle_pose.x)) - self.turtle_pose.theta

        while(True):
            
            dist=sqrt((self.turtle_pose.x-x_goal)**2+(self.turtle_pose.y-y_goal)**2)
            vel.linear.x=kp*(dist)+kd*(dist-eclud_dist_prev)+ki*e_sum


            steer=(atan2(y_goal - self.turtle_pose.y, x_goal - self.turtle_pose.x)-self.turtle_pose.theta)
            vel.angular.z=6*kp*steer+kd*(steer-steer_prev)+ki*steer_sum

            self.turtle_vel_pub.publish(vel)
            self.rate.sleep()

            eclud_dist_prev=dist
            e_sum = e_sum+eclud_dist_prev
            
            steer_prev=steer
            steer_sum=steer_sum+steer_prev
            if(dist<0.3):
                break
		




controlObj=FlytbaseTurtle()
kp=1
kd=0.01
ki=0.01
while not rospy.is_shutdown():
	controlObj.goToGoal(kp,kd,ki)
