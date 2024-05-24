#!/usr/bin/python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from pid import PIDController
from navigator import Navigator
import numpy as np


def init_node():
    # Create drone node
    rospy.init_node('surveillance_bot', anonymous=True)

def parse_coordinates(coords):
    return list(map(float, coords.split(',')))

def navigateTo(target=None):

    # -- Load in Map
    # Map start, target to image coords
    # Build PRM on that map
    # Generate path from PRM and convert to world points

    navigator = Navigator()

    start = None # Starting position
    goal = None # Ending position

    waypoints = [np.array([1,1]), np.array([2,1])]
    waypoint_count = 0
    

    pid_linear = PIDController(0.5, 0.1,0)
    pid_angular = PIDController(1, 0.1,0.5)


    # Follow PID Iteratively
    while (not rospy.is_shutdown() and waypoint_count < len(waypoints)):

        # Get the current position and angular velocity in world
        position, yaw = navigator.getCurrentState()
        position = np.array([position.x, position.y])
        print(position)

        # Compute target orientation based on current waypoint
        direction = waypoints[waypoint_count] - position
        print(direction)
        target_angular = np.arctan2(direction[1], direction[0])
        print(target_angular)

        if yaw < 0:
            yaw += 2*np.pi
        
        if target_angular < 0:
            target_angular += 2*np.pi
        
        if yaw - target_angular > np.pi:
            target_angular += 2*np.pi
        
        if target_angular - yaw > np.pi:
            yaw += 2*np.pi
        

        # Set the goals
        pid_linear.goal = waypoints[waypoint_count]
        pid_angular.goal = target_angular

        # Compute PID updates for linear and angular velocities
        update_linear = pid_linear.getUpdate(position)
        update_angular = pid_angular.getUpdate(yaw)
        print(update_linear)
        print(update_angular)

        # Publish velocities
        if abs(update_angular) >= 0.01:
            update_linear = [0,0]
        update_linear = [np.linalg.norm(update_linear), 0]
        navigator.publish_velocity(update_linear, update_angular)

        # Reached current waypoint, advance next
        if np.linalg.norm(position - waypoints[waypoint_count]) < 0.05:
            rospy.loginfo("Reached this wayppoint")
            waypoint_count += 1
            if waypoint_count < len(waypoints):
                pid_linear.goal = waypoints[waypoint_count]
    
    rospy.loginfo("Successfully arrived at location: %s ! ", position)

    stop_vel = np.zeros(3)



if __name__ == '__main__':
    try:
        init_node() # Create node
        # target = parse_coordinates(input())
        navigateTo()
        # except:
        #     print("Please specify the target location in the form x,y")
        
    except rospy.ROSInterruptException:
        pass
