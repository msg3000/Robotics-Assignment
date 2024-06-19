#!/usr/bin/python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from pid import PIDController
from navigator import Navigator
import numpy as np
from Mapping import WorldMapping
import cv2
from rrt import padding, GridMapFromImage, Node, RRT
import matplotlib.pyplot as plt

def init_node():
    # Create drone node
    rospy.init_node('surveillance_bot', anonymous=True)
    

def parse_coordinates(coords):
    print(coords)
    return list(map(float, coords))

def navigateTo(target=None):

    
    WorldMap=WorldMapping(0.05,[-13,-3,0])
   # GoalpixelCoords=WorldMap.world_to_pixel(target[0],target[1])
   
    image = WorldMap.image #Added this
    
    padded_binary_image = padding(image)
    print(padded_binary_image.shape)
    # Initialize the grid map from the padded binary image
    grid_map_obj = GridMapFromImage(padded_binary_image)
    
    # -- Load in Map
    # Map start, target to image coords
    # Build PRM on that map
    # Generate path from PRM and convert to world points

    navigator = Navigator()
    currentPos, _ =navigator.getCurrentState()
    print("CURRENT POS:", currentPos)
    CurrentpixelCoords=WorldMap.world_to_pixel(currentPos.x,currentPos.y)
    print("HEIGHT",WorldMap.height)
    print("WIDTH", WorldMap.width)
    print("CURRENT PIXEL COORDS: ",CurrentpixelCoords)
    # print("GOAL PIXEL COORDS: ",GoalpixelCoords)
    start = None # Starting position
    goal = None # Ending position

    rrt=RRT((0,0), target, grid_map_obj, WorldMap, 10)
    
    waypoints = rrt.build()
    waypoints_pixel = [WorldMap.world_to_pixel(x[0], x[1]) for x in waypoints]
    print("WAYPOINTS: ",waypoints)
    print("PIXEL WAYPOINtS: ",waypoints_pixel)
    plt.imshow(grid_map_obj.binary_image)

    for node in waypoints_pixel:
            if node == start:
                plt.plot(node, 'bo', markersize = 8, label = "Start")
                plt.text(node[0] - 1,node[1] + 1,'START')
            elif node == target:
                plt.plot(node, 'ro', label = "target", markersize = 8)
                plt.text(node[0] + 1,node[1] + 1,'GOAL')
            else:
                plt.plot(*node, marker = 'o', color = 'orange')
     
    
    for i, waypoint in enumerate(waypoints_pixel):
        if i != len(waypoints_pixel) - 1:
            start, end = waypoint, waypoints_pixel[i + 1]
            plt.plot([start[0], end[0]], [start[1], end[1]], 'g-')
    plt.show()
    
    waypoints = [np.array(waypoint) for waypoint in waypoints]
    
    waypoint_count = 0
    

    pid_linear = PIDController(2, 0,0)
    pid_angular = PIDController(1, 0.1,0.5)


    # Follow PID Iteratively
    while (not rospy.is_shutdown() and waypoint_count < len(waypoints)):

        # Get the current position and angular velocity in world
        position, yaw = navigator.getCurrentState()
        position = np.array([position.x, position.y])
       
        # Compute target orientation based on current waypoint
        direction = waypoints[waypoint_count] - position
       
        target_angular = np.arctan2(direction[1], direction[0])
        
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
        
        # Publish velocities
        if abs(update_angular) >= 0.1:
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
        target = parse_coordinates(input())
        
        navigateTo(target)
        # except:
        #     print("Please specify the target location in the form x,y")
        
    except rospy.ROSInterruptException:
        pass
