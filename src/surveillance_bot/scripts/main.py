#!/usr/bin/python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist


def init_node():
    # Create drone node
    rospy.init_node('drone', anonymous=True)

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

def move():
    move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    vel_msg = Twist()
    vel_msg.linear.x = 1 # Move forward

    while not rospy.is_shutdown():
        #Ensure that node has been subscribed to by takeoff
        is_connected = move_pub.get_num_connections() > 0
        if is_connected:
            move_pub.publish(vel_msg)
            rospy.loginfo('Flying in the air')
            break
    rospy.sleep(rospy.Duration(3.0)) # Fly for 3 seconds
    
    vel_msg.linear.x = 0 # Halt the drone
    move_pub.publish(vel_msg) 
    rospy.sleep(rospy.Duration(1)) # Allow time to halt before landing




    

if __name__ == '__main__':
    try:
        init_node() # Create node
    except rospy.ROSInterruptException:
        pass
