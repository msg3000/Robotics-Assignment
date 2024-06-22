#!/usr/bin/python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from pid import PIDController
from navigator import Navigator
import numpy as np
from mapping import WorldMapping
from rrt import padding, GridMapFromImage, Node, RRT
import matplotlib.pyplot as plt

def init_node():
    # Create drone node
    rospy.init_node('surveillance_bot', anonymous=True)
    

def parse_coordinates(coords):
    # Parse inputs correctly
    return list(map(float, coords))

def navigateTo(target=None):
    """ 
    Main operating logic of surveillance bot.
    Takes in a specified target in the world and navigates there
    """

    if target is None:
        return
    
    # Instantiate navigator for state service info and publishing
    navigator = Navigator()

    # Extract current state information
    currentPos, _ =navigator.getCurrentState()
    start = currentPos # Starting position
    
    print("Beginning search for a path from ({},{}) to ({},{})".format(round(start.x,2), round(start.y,2), target[0], target[1]))
    print("Instantiating world information and map...")
    # Instantiate world map with pixel mapping of supplied map
    WorldMap=WorldMapping(0.05,[-17,-14.4,0])
   
    # Extract image, binarize and apply padding
    image = WorldMap.image
    padded_binary_image = padding(image)

    # Initialize the grid map from the padded binary image
    grid_map_obj = GridMapFromImage(padded_binary_image)
    plt.imshow(1 - grid_map_obj.binary_image, cmap = plt.cm.gray)
    

    print("Computing path from ({}, {}) to ({}, {})...".format(round(start.x,2), round(start.y,2), target[0], target[1]))
    # Instantiate RRT with current start, goal, grid map and step size parameters
    rrt=RRT((start.x,start.y), target, grid_map_obj, WorldMap, 15, 30)
    
    # Build path from start to goal
    waypoints = rrt.build()

    if waypoints is None:
        print("Failed to find a path to target location \n. TERMINATING")
        return
    
    # Visualisation
    #rrt.visualize(rrt.start)
    _ = rrt.get_path(display=True)
    plt.show()
    
    print("Successfully found a path !")
    print("Beginning navigation from ({}, {})...".format(round(start.x,2), round(start.y,2), target[0], target[1]))
    waypoints = [np.array(waypoint) for waypoint in waypoints]
    waypoint_count = 0
    
    # Initialise pid controllers for linear and angular
    pid_linear = PIDController(0.5, 0,0)
    pid_angular = PIDController(2, 0.1,0.1)


    # Follow PID Iteratively
    while (not rospy.is_shutdown() and waypoint_count < len(waypoints)):

        # Get the current position and angular velocity in world
        position, yaw = navigator.getCurrentState()
        position = np.array([position.x, position.y])
       
        # Compute target orientation based on current waypoint
        direction = waypoints[waypoint_count] - position
       
        # Target angular in direction of next waypoint
        target_angular = np.arctan2(direction[1], direction[0])
        
        # Ensures angles are normalised between 0 and pi, and that no larger than pi
        if yaw < 0:
            yaw += 2*np.pi
        
        if target_angular < 0:
            target_angular += 2*np.pi
        
        if yaw - target_angular > np.pi:
            target_angular += 2*np.pi
        
        if target_angular - yaw > np.pi:
            yaw += 2*np.pi
        
        # Set the goals for pid
        pid_linear.goal = waypoints[waypoint_count]
        pid_angular.goal = target_angular

        # If angular deviation is significant, orient to goal direction first
        if abs(target_angular - yaw) >= 0.2:
            update_linear = [0,0]
            while True:
                _, yaw = navigator.getCurrentState()
                if yaw < 0:
                    yaw += 2*np.pi
                if yaw - target_angular > np.pi:
                    target_angular += 2*np.pi
                if target_angular - yaw > np.pi:
                    yaw += 2*np.pi
                update_angular = pid_angular.getUpdate(yaw)
                navigator.publish_velocity(update_linear, update_angular)

                if abs(yaw - target_angular) < 0.02:
                    break


        # Compute PID updates for linear and angular velocities
        update_linear = pid_linear.getUpdate(position)
        update_angular = pid_angular.getUpdate(yaw)


            
        update_linear = [np.linalg.norm(update_linear), 0]

        # If updating linearly, prevent angular deviations
        # if (update_linear[0]!=0):
        #     update_angular=0    
        navigator.publish_velocity(update_linear, update_angular)

        # Reached current waypoint, advance next
        if np.linalg.norm(position - waypoints[waypoint_count]) < 0.05:
            print("Reached waypoint : ({}, {})".format(round(waypoints[waypoint_count][0],2), round(waypoints[waypoint_count][1],2)))
            waypoint_count += 1
            if waypoint_count < len(waypoints):
                pid_linear.goal = waypoints[waypoint_count]
    
    print("SUCCESSFULLY ARRIVED AT GOAL LOCATION: ({},{}) ! ".format(target[0], target[1]))



if __name__ == '__main__':
    try:
        init_node() # Create node
        try:
            target = parse_coordinates(input("Enter the surveillance destination of the form x,y: "))  
        except:
            print("Please specify the target location in the form x,y")
        navigateTo(target)
    except rospy.ROSInterruptException:
        print("An unexpected error has occurred")