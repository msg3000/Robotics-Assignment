# Handles state information and velocity publishing

import rospy
import sys
import rospy
import time
import numpy as np
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion

class Navigator:
    def __init__(self):
        
        # Publish velocity
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        # Extract model state information
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.velocity_message = Twist()


    def getCurrentState(self):
        """
        Extracts current coordinates and orientation from world
        """
        state = self.get_model_state('mobile_base', "")
        
        position, orientation = state.pose.position, state.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return (position, yaw)
        

    def publish_velocity(self, linear_vel , angular_vel, angular = False):
        """
        Publish velocity to turtlebot, linear along x and angular around z
        """

        rate = rospy.Rate(10)
    
        self.velocity_message.linear.x = linear_vel
        self.velocity_message.angular.z = angular_vel
        
        while not rospy.is_shutdown():
            #Ensure that node has been subscribed to by takeoff
            is_connected = self.velocity_publisher.get_num_connections() > 0
            if is_connected:
                self.velocity_publisher.publish(self.velocity_message)
                break
            rate.sleep()

    

        
        