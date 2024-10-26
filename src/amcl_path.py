#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path

class AMCLPathPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('amcl_path_publisher')
        rospy.set_param('/use_sim_time', True)
        
        # Publisher for the Path message
        self.path_publisher = rospy.Publisher('/robot_path_amcl', Path, queue_size=10)
        
        # Initialize the Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"  # AMCL typically operates in the "map" frame
        
        # Subscribe to the AMCL pose data
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        
        # Timer to publish the path at a fixed rate
        self.rate = rospy.Rate(10)  # 10 Hz for path publishing
        self.publish_path()
        
    def amcl_pose_callback(self, amcl_pose_msg):
        """Callback function to handle incoming AMCL PoseWithCovarianceStamped messages."""
        # Create a PoseStamped message from the AMCL pose message
        pose_stamped = PoseStamped()
        pose_stamped.header = amcl_pose_msg.header
        pose_stamped.pose = amcl_pose_msg.pose.pose  # Use the pose from AMCL PoseWithCovarianceStamped
        
        # Add the pose to the path
        self.path_msg.poses.append(pose_stamped)

        rospy.loginfo("Received /amcl_pose and updated path.")
        
    def publish_path(self):
        """Publish the accumulated path at a fixed rate."""
        while not rospy.is_shutdown():
            self.path_msg.header.stamp = rospy.Time.now()  # Update the timestamp
            self.path_publisher.publish(self.path_msg)  # Publish the path
            self.rate.sleep()  # Sleep to maintain the loop rate

if __name__ == '__main__':
    try:
        amcl_path_publisher = AMCLPathPublisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated")