#!/usr/bin/env python3

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from Simulation_pkg.msg import PositionVelocity
import threading
import sys

# Global variables
position_velocity_pub = None
pos_vel_msg = PositionVelocity()

# Action client for interacting with the Action Server
ac = None

# Locks for synchronizing access between threads
ac_lock = threading.Lock()

# Status flags
goal_reached = False
request_coordinates = False
goal_canceled = False

# Callback for odometry data
def odom_callback(msg):
    # Extract position and velocity from odometry data
    pos_vel_msg.x = msg.pose.pose.position.x
    pos_vel_msg.y = msg.pose.pose.position.y
    pos_vel_msg.vel_x = msg.twist.twist.linear.x
    pos_vel_msg.vel_z = msg.twist.twist.angular.z
    
    # Publish custom message
    position_velocity_pub.publish(pos_vel_msg)

# Function to monitor goal status
def monitor_goal_status():
    global goal_reached, request_coordinates
    
    # Subscriber
    rospy.Subscriber("/odom", Odometry, odom_callback)
    
    rate = rospy.Rate(10)  # Monitoring frequency
    while not rospy.is_shutdown():
        with ac_lock:
            if ac:
                state = ac.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED and not goal_reached:
                    rospy.loginfo("Target reached!")
                    rospy.loginfo("Type 'set' to set a goal or 'cancel' to cancel:")
                    goal_reached = True  # Indicate that the goal was reached
                    request_coordinates = False  # Allow new goals to be set
                elif state == actionlib.GoalStatus.ABORTED:
                    rospy.logwarn("Target could not be reached. Please set a new goal.")
                    goal_reached = True  # Allow setting a new goal
                    request_coordinates = False
        
        rate.sleep()

def main():
    global position_velocity_pub, ac, goal_reached, request_coordinates
    
    rospy.init_node('action_client_node')
    
    # Publisher
    position_velocity_pub = rospy.Publisher("position_velocity", PositionVelocity, queue_size=10)
    
    # Action client
    ac = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    
    rospy.loginfo("Waiting for Action server...")
    ac.wait_for_server()
    rospy.loginfo("Action server available.")
    
    # Start monitor thread
    monitor_thread = threading.Thread(target=monitor_goal_status)
    monitor_thread.daemon = True
    monitor_thread.start()
    
    # Main loop for user interaction
    while not rospy.is_shutdown():
        if not request_coordinates:
            rospy.loginfo("Type 'set' to set a goal or 'cancel' to cancel:")
            try:
                command = input().strip()
                
                if command == "set":
                    # Request and send target coordinates
                    rospy.loginfo("Enter target coordinates on the same line separated by space (x y):")
                    coords = input().strip()
                    
                    # Parse coordinates
                    try:
                        x, y = map(float, coords.split())
                        
                        # Ensure both x and y are valid numbers (not NaN or infinity)
                        if not (isinstance(x, float) and isinstance(y, float) and 
                                x == x and y == y):  # Check for NaN
                            rospy.logerr("Invalid input. Both x and y must be finite numbers.")
                            continue
                        
                        # Update the msg with the new target
                        pos_vel_msg.target_x = x
                        pos_vel_msg.target_y = y
                        
                        # Create and send goal
                        goal = PlanningGoal()
                        goal.target_pose.header.frame_id = "map"  # Ensure a valid frame
                        goal.target_pose.header.stamp = rospy.Time.now()
                        goal.target_pose.pose.position.x = x
                        goal.target_pose.pose.position.y = y
                        goal.target_pose.pose.orientation.w = 1.0  # Default orientation
                        
                        with ac_lock:
                            ac.send_goal(goal)
                        
                        goal_reached = False        # Reset goal reached flag
                        rospy.loginfo(f"Goal sent to ({x:.2f}, {y:.2f}).")
                        
                    except ValueError:
                        rospy.logerr("Invalid input. Enter two numbers (x y) separated by a space.")
                        
                elif command == "cancel":
                    # Cancel the current goal
                    with ac_lock:
                        ac.cancel_goal()
                    rospy.loginfo("Goal canceled.")
                    request_coordinates = False  # Allow setting a new goal
                    
                else:
                    rospy.logwarn("Unrecognized command. Type 'set' or 'cancel'.")
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                break
    
    # Wait for monitor thread to finish
    rospy.signal_shutdown("User exit")
    monitor_thread.join(timeout=1.0)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
