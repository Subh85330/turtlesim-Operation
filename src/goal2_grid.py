#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi

class FlytbaseTurtle():
    def __init__(self):
        rospy.init_node('goal1_node')
        self.turtle_pose = Pose()
        self.turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        self.rate=rospy.Rate(10)
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
            self.rate.sleep()
            eclud_dist_prev=dist
            
            
            
            if(dist<0.3):
                break
		


    def accelLimits(self,vel_initial,vel_final,k_accel,k_decel):  
        dt = 5
        if(vel_final.linear.x - vel_initial.linear.x > 0):
             k=k_accel
             a_max_linear = 1.5
             a_max_angular = 6
        else:
             k=k_decel
             a_max_linear = -1.5
             a_max_angular = -6
        current_vel = Twist()
        dv = k*(vel_final.linear.x - vel_initial.linear.x)
        if(abs(dv/dt)>abs(a_max_linear)):
             dv=dt*a_max_linear			


        d_omega = 5*k*(vel_final.angular.z - vel_initial.angular.z)
        if(abs(d_omega/dt)>abs(a_max_angular)):
             d_omega=dt*a_max_angular

        current_vel.linear.x = vel_initial.linear.x + dv
        current_vel.angular.z = vel_initial.angular.z + d_omega
        self.turtle_vel_pub.publish(current_vel)
        return current_vel

    def rotateTurtle(self, theta):
         vel = Twist()
         while((abs((theta*pi/180)-(self.turtle_pose.theta)))>0.03):
              vel.angular.z = (theta*pi/180) - (self.turtle_pose.theta)
              self.turtle_vel_pub.publish(vel)


    def grid(self,kp,kd,ki,k_accel,k_decel):
         grid_points = [(1,1,0),(10,1,90),(10,3,180),(1,3,90),(1,5,0),(10,5,90),(10,7,180),(1,7,90),(1,9,0),(10,9,0)]

         for i in range(len(grid_points)):
              x_goal = grid_points[i][0]
              y_goal = grid_points[i][1]
              angle = grid_points[i][2]
              self.goToGoal(kp,kd,ki,k_accel, k_decel, x_goal, y_goal)
              self.rotateTurtle(angle)
            


controlObj=FlytbaseTurtle()
kp=1
kd=0.1
ki=0.0001
k_accel = 0.1
k_decel = 0.3
while not rospy.is_shutdown():
	controlObj.grid(kp,kd,ki,k_accel,k_decel)
