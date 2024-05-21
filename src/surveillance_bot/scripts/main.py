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

    waypoints = [np.array([5,5])]
    waypoint_count = 0
    

    pid_linear = PIDController(1,1,1)
    pid_angular = PIDController(1,1,1)


    # Follow PID Iteratively
    while (not rospy.is_shutdown() and waypoint_count < len(waypoints)):

        # Get the current position and angular velocity in world
        position, yaw = navigator.get_model_state()
        position = np.array([position.x, position.y])

        # Compute target orientation based on current waypoint
        direction = waypoints[waypoint_count] - position
        target_angular = np.arctan2(direction[1], direction[0])

        # Set the goals
        pid_linear.goal = waypoints[waypoint_count]
        pid_angular.goal = target_angular

        # Compute PID updates for linear and angular velocities
        update_linear = pid_linear.getUpdate(position)
        update_angular = pid_angular.getUpdate(yaw)

        # Publish velocities
        navigator.publish_velocity(update_linear, update_angular)

        # Reached current waypoint, advance next
        if np.linalg.norm(position - waypoints[waypoint_count]) < 0.05:
            waypoint_count += 1
            if waypoint_count < len(waypoints):
                update_linear.goal = waypoints[waypoint_count]
    
    rospy.loginfo("Successfully arrived at location: %s ! ", position)

    stop_vel = np.zeros(3)



if __name__ == '__main__':
    try:
        init_node() # Create node
        try:
            target = parse_coordinates(input())
            navigateTo()
        except:
            print("Please specify the target location in the form x,y")
        
    except rospy.ROSInterruptException:
        pass
