#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_conversions

class DynamicTFBroadcaster:
    def __init__(self):
        rospy.init_node('dynamic_tf_broadcaster')

        # Transform Broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        
        # Subscriber for Odometry messages
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # Create a TransformStamped message
        transform = TransformStamped()

        # Set header
        # transform.header.stamp = msg.header.stamp
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        # Set translation (position)
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        # Set rotation (orientation)
        transform.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.br.sendTransform(transform)


if __name__ == '__main__':
    try:
        DynamicTFBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
