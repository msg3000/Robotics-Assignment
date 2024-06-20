#!/usr/bin/python
# Implements PID for general objects.
import sys
import rospy
import time
import numpy as np
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState

class PIDController:
    def __init__(self, kp, ki, kd):
        

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_time = time.time()
        self.prev_err = 0
        self.err_i = 0

    def set_goal(self, goal):
        self.goal = goal

    def getUpdate(self, current_val):

        error = self.goal - current_val
        dt = time.time() - self.prev_time
        self.err_i += error * dt
        derror = (error - self.prev_err)/dt if dt > 0 else 0
    
        update = self.kp * error + self.ki * self.err_i + self.kd * derror

        self.prev_err = error
        self.prev_time = time.time()

        return update
    



