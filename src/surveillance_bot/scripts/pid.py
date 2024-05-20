#!/usr/bin/python
import sys
import rospy
import time
import numpy as np
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState

class PIDController:
    def __init__(self, goal, kp, ki, kd):
        self.goal = goal

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_time = 0
        self.prev_err = 0
        self.err_i = 0

    def getUpdate(self, current_val):

        error = self.goal - current_val
        dt = time.time() - self.prev_time
        self.err_i += error * dt
        derror = (error - self.prev_err)/dt if dt > 0 else 0
    
        update = self.kp * error + self.ki * self.err_i + self.kd * derror

        self.prev_err = error
        self.prev_time = time.time()

        return update
    
def init_node():
    # Create drone node
    rospy.init_node('pid_controller', anonymous=True)

def takeoff():
    takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #Ensure that node has been subscribed to by takeoff
        is_connected = takeoff_pub.get_num_connections() > 0
        if is_connected:
            takeoff_pub.publish()
            rospy.loginfo('Took off')
            break
        rate.sleep()

def land():
    land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        connections = land_pub.get_num_connections()
        if connections > 0:
            land_pub.publish()
            rospy.loginfo('Landed')
            break
        rate.sleep()

def publish_velocity(vel_pub, linear_vel , angular_vel):
    vel_msg = Twist()

    vel_msg.linear.x = linear_vel[0]
    vel_msg.linear.y = linear_vel[1]
    vel_msg.linear.z = linear_vel[2]

    vel_msg.angular.z = angular_vel

    vel_pub.publish(vel_msg)



def moveToTarget(goal):
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    linear_controller = PIDController(goal, 0.5, 0, 0)
    angular_controller = PIDController(0, 0.5, 0, 0)

    pos = get_model_state('quadrotor', "")
    current_pos = np.array([pos.pose.position.x, pos.pose.position.y, pos.pose.position.z])

    while (np.linalg.norm(current_pos - goal) >= 0.05):

        # Get the current position and angular velocity
        pos = get_model_state('quadrotor', "")
        current_pos = np.array([pos.pose.position.x, pos.pose.position.y, pos.pose.position.z])
        angular_z = pos.twist.angular.z

        uLinear =  linear_controller.getUpdate(current_pos)
        uAngular = angular_controller.getUpdate(angular_z)

        publish_velocity(vel_pub, uLinear, uAngular)
    
    rospy.loginfo("Successfully arrived at location: %s ! ", current_pos)

    stop_vel = np.zeros(3)
    publish_velocity(vel_pub, stop_vel, 0)


if __name__ == '__main__':
    try:
        try:
            target = list(map(float, sys.argv[1].split(',')))
            print(target)
        except:
            print("Please specify the target location in the form x,y,z")
        init_node() # Create node
        takeoff() # Takeoff
        rospy.sleep(rospy.Duration(2)) # Allow time before moving
        moveToTarget(np.array(target))
        rospy.sleep(rospy.Duration(2))
        land() # Land
    except rospy.ROSInterruptException:
        pass
