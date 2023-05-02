#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi
from numpy.random import normal
import time

class FlytbaseTurtle():
    def __init__(self):
        rospy.init_node('goal3_circle_node')
        self.turtle_pose = Pose()
        self.turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        self.rt_real_pose_pub = rospy.Publisher('/rt_real_pose',Pose,queue_size=10)
        self.rt_noidy_pose_pub = rospy.Publisher('/rt_noisy_pose',Pose,queue_size=10)
        self.vel_rate=rospy.Rate(10)
        self.pose_rate =rospy.Rate(0.2)
        self.turtle_pose_sub=rospy.Subscriber('/turtle1/pose',Pose,self.dataRecvCallback)
    
    def dataRecvCallback(self,turtle_pose_data_recv):
        self.turtle_pose=turtle_pose_data_recv

    def goToGoal(self,kp,kd,ki,k_accel,k_decel, x_goal, y_goal):
        vel=Twist()
        
        e_sum = 0
        eclud_dist_prev = sqrt((self.turtle_pose.x-x_goal)**2 + (self.turtle_pose.y - y_goal)**2)

        
        steer_sum = 0
        steer_prev=(atan2(y_goal-self.turtle_pose.y, x_goal-self.turtle_pose.x)) - self.turtle_pose.theta

        vel_prev = Twist()
        while(True):
            e_sum = e_sum+eclud_dist_prev
            dist=sqrt((self.turtle_pose.x-x_goal)**2+(self.turtle_pose.y-y_goal)**2)
            vel.linear.x=kp*(dist)+kd*(dist-eclud_dist_prev)+ki*e_sum


            steer=(atan2(y_goal - self.turtle_pose.y, x_goal - self.turtle_pose.x)-self.turtle_pose.theta)
            vel.angular.z=6*kp*steer#+kd*(steer-steer_prev)+ki*steer_sum

            vel_prev = self.accelLimits(vel_prev, vel, k_accel, k_decel)
            self.vel_rate.sleep()
            eclud_dist_prev=dist
            
            
            
            if(dist<0.3):
                break
		
    def accelLimits(self,vel_initial,vel_final,k_accel,k_decel):  
        if(vel_final.linear.x - vel_initial.linear.x > 0):
             k=k_accel
        else:
             k=k_decel
        current_vel = Twist()
        dv = k*(vel_final.linear.x - vel_initial.linear.x)
        d_omega = 5*k*(vel_final.angular.z - vel_initial.angular.z)

        current_vel.linear.x = vel_initial.linear.x + dv
        current_vel.angular.z = vel_initial.angular.z + d_omega
        self.turtle_vel_pub.publish(current_vel)
        return current_vel

    def rotateTurtle(self, theta):
         vel = Twist()
         while((abs((theta*pi/180)-(self.turtle_pose.theta)))>0.03):
              vel.angular.z = (theta*pi/180) - (self.turtle_pose.theta)
              self.turtle_vel_pub.publish(vel)

    def moveOnCircle(self,linear_velocity,radius,k_accel,k_decel):
         vel_cmd = Twist()
         vel_cmd.linear.x = linear_velocity
         vel_cmd.angular.z = linear_velocity/radius
         t_start = time.time()
         vel_prev = Twist()
         while(not rospy.is_shutdown()):
              vel_prev = self.accelLimits(vel_prev, vel_cmd,k_accel,k_decel)
              self.vel_rate.sleep()
              if(time.time()-t_start>5):
                   self.rt_real_pose_pub.publish(self.turtle_pose)
                   self.noisyPose()
                   t_start = time.time()
    
    def noisyPose(self):
         noisy_pose = Pose()
         noisy_pose.x = self.turtle_pose.x + normal(0,2)
         noisy_pose.y = self.turtle_pose.y + normal(0,2)
         noisy_pose.theta = self.turtle_pose.theta + normal(0,0.1)
         noisy_pose.angular_velocity = self.turtle_pose.angular_velocity + normal(0,0.1)
         noisy_pose.linear_velocity = self.turtle_pose.linear_velocity + normal(0,0.1)
         self.rt_noidy_pose_pub.publish(noisy_pose)

controlObj=FlytbaseTurtle()
kp=1
kd=0.1
ki=0.0001
k_accel = 0.1
k_decel = 0.3
v=2
r=2
while not rospy.is_shutdown():
	controlObj.moveOnCircle(v,r,k_accel,k_decel)
