#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

def movebase_client(x, y, theta):
    # Creates an action client to move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Waits until the action server has started up and started listening for goals
    client.wait_for_server()

    # Creates a new goal to send to move_base
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Define the position and orientation (x, y, theta) in the map frame
    goal.target_pose.pose.position = Point(x, y, 0.0)
    quat = quaternion_from_euler(0, 0, theta)
    goal.target_pose.pose.orientation = Quaternion(*quat)

    # Send the goal to the action server and wait for result
    client.send_goal(goal)
    wait = client.wait_for_result()

    if wait:
        return client.get_result()
    else:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
        return None

if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_nav')

        waypoints = [
            #(-0.395, 1.74, 0.00304),  # Waypoint 1
            #(0.606, 0.406, 0.00397),  # Waypoint 2
            #(-0.478, -1.63, 0),  # Waypoint 3
            #(-1.79, 0.458, 0.0)    # Waypoint 4

            (-0.36, 1.85, 0),
            (0.563, -0.54, 0),
            (1.88, 0.458, 0)
        ]

        for waypoint in waypoints:
            x, y, theta = waypoint
            result = movebase_client(x, y, theta)
            if result:
                rospy.loginfo(f"Reached waypoint: {waypoint}")
            else:
                rospy.loginfo(f"Failed to reach waypoint: {waypoint}")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
