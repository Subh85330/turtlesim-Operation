#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi
from numpy.random import normal
import numpy as np
import math
import time
predicated_points = []
class FlytbaseTurtle():
    def __init__(self):
        rospy.init_node('goal4_PT_node')
        self.turtle_pose = Pose()
        self.turtle_vel_pub = rospy.Publisher('/turtle2/cmd_vel',Twist,queue_size=10)
        self.rt_real_pose_sub = rospy.Subscriber('/rt_real_pose',Pose,self.chaseRT)
        self.vel_rate=rospy.Rate(10)
        self.turtle_pose_sub=rospy.Subscriber('/turtle2/pose',Pose,self.dataRecvCallback)
        self.v_max = 2
    def dataRecvCallback(self,turtle_pose_data_recv):
        self.turtle_pose=turtle_pose_data_recv

    def goToGoal(self,kp,kd,ki,k_accel,k_decel, x_goal, y_goal):
        vel=Twist()
        
        e_sum = 0
        eclud_dist_prev = sqrt((self.turtle_pose.x-x_goal)**2 + (self.turtle_pose.y - y_goal)**2)

        
        steer_sum = 0
        steer_prev=(atan2(y_goal-self.turtle_pose.y, x_goal-self.turtle_pose.x)) - self.turtle_pose.theta

        vel_prev = Twist()
        t_f = time.time()
        while(True):
            e_sum = e_sum+eclud_dist_prev
            dist=sqrt((self.turtle_pose.x-x_goal)**2+(self.turtle_pose.y-y_goal)**2)
            vel.linear.x=kp*(dist)+kd*(dist-eclud_dist_prev)+ki*e_sum


            steer=(atan2(y_goal - self.turtle_pose.y, x_goal - self.turtle_pose.x)-self.turtle_pose.theta)
            vel.angular.z=6*kp*steer#+kd*(steer-steer_prev)+ki*steer_sum

            vel_prev, t_f = self.accelLimits(vel_prev, vel, t_f, k_accel, k_decel)
            self.vel_rate.sleep()
            eclud_dist_prev=dist
            
            
            
            if(dist<0.3):
                break
		
    def accelLimits(self,vel_initial,vel_final,time_send,k_accel,k_decel):
        if(vel_final.linear.x - vel_initial.linear.x > 0):
             k=k_accel
             a_max_linear = 1.5
             a_max_angular = 5
        else:
             k=k_decel
             a_max_linear = -1.5
             a_max_angular = -5
        current_vel = Twist()
        dv = k*(vel_final.linear.x - vel_initial.linear.x)
        t = time.time()
        dt = t - time_send
        if(abs(dv/dt)>abs(a_max_linear)):
             dv=dt*a_max_linear			


        d_omega = 1*k*(vel_final.angular.z - vel_initial.angular.z)
        if(abs(d_omega/dt)>abs(a_max_angular)):
             d_omega=dt*a_max_angular


        current_vel.linear.x = vel_initial.linear.x + dv
        if(current_vel.linear.x  > self.v_max):
             current_vel.linear.x  = self.v_max
        current_vel.angular.z = vel_initial.angular.z + d_omega
        self.turtle_vel_pub.publish(current_vel)
        timeL = time.time()
        return (current_vel, timeL)

    def find_circle_center_radius(self, p1, p2, p3):
          x1, y1 = p1
          x2, y2 = p2
          x3, y3 = p3
          # Calculate the midpoints
          x12 = (x1 + x2) / 2
          y12 = (y1 + y2) / 2
          x23 = (x2 + x3) / 2
          y23 = (y2 + y3) / 2

          # Calculate the slopes
          m1 = (y2 - y1) / (x2 - x1)
          m2 = (y3 - y2) / (x3 - x2)

          # Calculate the perpendicular slopes
          m1_perpendicular = -1 / m1
          m2_perpendicular = -1 / m2

          # Calculate the y-intercepts
          c1 = y12 - m1_perpendicular * x12
          c2 = y23 - m2_perpendicular * x23

          # Calculate the x-coordinate of the center
          center_x = (c2 - c1) / (m1_perpendicular - m2_perpendicular)

          # Calculate the y-coordinate of the center
          center_y = m1_perpendicular * center_x + c1

          # Calculate the radius
          radius = math.sqrt((x1 - center_x)**2 + (y1 - center_y)**2)
          center = (center_x, center_y)
          return (center, radius)

    def calculate_next_point(self,prev_point1, prev_point2, center, radius):
          # Extract the coordinates of the previous points
          x1, y1 = prev_point1
          x2, y2 = prev_point2

          # Calculate the vector from the center to the previous point 1
          prev_vec1_x = x1 - center[0]
          prev_vec1_y = y1 - center[1]

          # Calculate the vector from the center to the previous point 2
          prev_vec2_x = x2 - center[0]
          prev_vec2_y = y2 - center[1]

          # Calculate the angles of the previous points with respect to the center
          prev_angle1 = math.atan2(prev_vec1_y, prev_vec1_x)
          prev_angle2 = math.atan2(prev_vec2_y, prev_vec2_x)

          # Calculate the angle increment
          angle_increment = prev_angle2 - prev_angle1

          # Calculate the new angle for the next point
          next_angle = prev_angle2 + angle_increment

          # Calculate the coordinates of the next point
          next_point_x = center[0] + radius * math.cos(next_angle)
          next_point_y = center[1] + radius * math.sin(next_angle)

          return (next_point_x, next_point_y)
    

    def distance(self,pointX, pointY):
         return(sqrt((self.turtle_pose.x-pointX)**2+(self.turtle_pose.y-pointY)**2))

    def chaseRT(self, RT_pose):
         print("chaseRT Called")
         kp=1
         kd=0.1
         ki=0.0001
         k_accel = 0.1
         k_decel = 0.3
         global predicated_points
         dist = self.distance(RT_pose.x, RT_pose.y)
         print("Dist", dist)
         if(dist<3):
               zero_vel=Twist()
               self.turtle_vel_pub.publish(zero_vel)
               rospy.set_param('IsCaught',True)
               print("Distance less than 3. Caught RT\n")
               self.rt_real_pose_sub.unregister()
               
         else:
               futurePredicatedPose = Pose()
               if(np.shape(predicated_points)[0]<3):
                    predicated_points.append([RT_pose.x,RT_pose.y])
                    print(predicated_points)
               if(np.shape(predicated_points)[0]>=3):
                    center, radius = self.find_circle_center_radius(predicated_points[0],predicated_points[1],predicated_points[2])
                    nextPoint = self.calculate_next_point(predicated_points[1],predicated_points[2],center,radius)
                    
                    timeFactor = 1
                    while(True):
                         print("Chasing>..........")
                         if(self.distance(nextPoint[0],nextPoint[1])/self.v_max>(5*timeFactor)):
                              timeFactor += 1
                              predicated_points.append(nextPoint)
                              next_point=self.next_point(a[-1],a[-2],center,radius)
                              timeFactor +=1

                         else:
                              self.goToGoal(kp,kd,ki,k_accel,k_decel,nextPoint[0],nextPoint[1])
                              break

controlObj=FlytbaseTurtle()
rospy.spin()
