#!/usr/bin/python3
#
# Copyright (c) 2022-2024, ETH Zurich, Matias Mattamala, Jonas Frey.
# All rights reserved. Licensed under the MIT license.
# See LICENSE file in the project root for details.
#
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from wild_visual_navigation_msgs.msg import RobotState, CustomState
import rospy
import tf2_ros
import numpy as np
import tf.transformations

def jackal_msg_callback(robot_state, return_msg=False):

    robot_state.pose.pose.position.x = robot_state.pose.pose.position.x - 0.3
    robot_state.pose.pose.position.z = 0.3
    robot_state_msg = RobotState()

    # For RobotState msg
    robot_state_msg.header = robot_state.header

    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"

    transform.transform.translation.x = robot_state.pose.pose.position.x
    transform.transform.translation.y = robot_state.pose.pose.position.y
    transform.transform.translation.z = robot_state.pose.pose.position.z

    # Set rotation (orientation)
    transform.transform.rotation = robot_state.pose.pose.orientation

    # Broadcast the odom -> base_link transform
    br.sendTransform(transform)

    # Add base_link -> camera_color_optical_frame transform
    camera_transform = TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "base_link"
    camera_transform.child_frame_id = "camera_color_optical_frame"

    # Camera transform matrix
    # camera_matrix = np.array([
    #     [0.0796631, 0.00164398, 0.99682, 0.115613],
    #     [-0.996616, 0.0204396, 0.0796131, -0.194337],
    #     [-0.0202437, -0.99979, 0.0032667, 0.122819],
    #     [0, 0, 0, 1]
    # ])
    
    camera_matrix = np.array([
        [0.00330765, 0.00592764, 0.999977, 0.50412],
        [-0.999994, -0.000849364, 0.00331274, 0.0261041],
        [0.000868981, -0.999982, 0.0059248, 0.2554703],
        [0, 0, 0, 1]
    ])

    # Extract translation and rotation from matrix
    translation = camera_matrix[:3, 3]
    quaternion = tf.transformations.quaternion_from_matrix(camera_matrix)

    # Set translation
    camera_transform.transform.translation.x = translation[0]
    camera_transform.transform.translation.y = translation[1]
    camera_transform.transform.translation.z = translation[2]

    # Set rotation
    camera_transform.transform.rotation.x = quaternion[0]
    camera_transform.transform.rotation.y = quaternion[1]
    camera_transform.transform.rotation.z = quaternion[2]
    camera_transform.transform.rotation.w = quaternion[3]

    # Broadcast the base_link -> camera_color_optical_frame transform
    br.sendTransform(camera_transform)

    # Extract pose
    robot_state_msg.pose.header = robot_state.header
    robot_state_msg.pose.header.frame_id = "base_link"
    robot_state_msg.pose.header.stamp = rospy.Time.now()
    robot_state_msg.pose.pose = robot_state.pose.pose

    # Extract twist
    robot_state_msg.twist.header = robot_state.header
    robot_state_msg.twist.header.frame_id = robot_state.child_frame_id
    robot_state_msg.twist.twist = robot_state.twist.twist

    vector_state = CustomState()
    vector_state.name = "vector_state"
    vector_state.dim = 7 + 6  # + 4 * 12
    vector_state.values = [0] * vector_state.dim
    vector_state.labels = [""] * vector_state.dim
    vector_state.values = [0] * vector_state.dim
    robot_state_msg.states.append(vector_state)

    robot_state_msg.states[0].values[0] = robot_state_msg.pose.pose.position.x
    robot_state_msg.states[0].values[1] = robot_state_msg.pose.pose.position.y
    robot_state_msg.states[0].values[2] = robot_state_msg.pose.pose.position.z
    robot_state_msg.states[0].values[3] = robot_state_msg.pose.pose.orientation.x
    robot_state_msg.states[0].values[4] = robot_state_msg.pose.pose.orientation.y
    robot_state_msg.states[0].values[5] = robot_state_msg.pose.pose.orientation.z
    robot_state_msg.states[0].values[6] = robot_state_msg.pose.pose.orientation.w
    robot_state_msg.states[0].values[7] = robot_state_msg.twist.twist.linear.x
    robot_state_msg.states[0].values[8] = robot_state_msg.twist.twist.linear.y
    robot_state_msg.states[0].values[9] = robot_state_msg.twist.twist.linear.z
    robot_state_msg.states[0].values[10] = robot_state_msg.twist.twist.angular.x
    robot_state_msg.states[0].values[11] = robot_state_msg.twist.twist.angular.y
    robot_state_msg.states[0].values[12] = robot_state_msg.twist.twist.angular.z

    for i, x in enumerate(["tx", "ty", "tz", "qx", "qy", "qz", "qw", "vx", "vy", "vz", "wx", "wy", "wz"]):
        robot_state_msg.states[0].labels[i] = x

    if return_msg:
        return robot_state_msg
    # Publish
    robot_state_pub.publish(robot_state_msg)


def twist_msg_callback(msg):
    ts = rospy.Time.now()
    out_msg = TwistStamped()
    out_msg.header.stamp = ts
    out_msg.header.frame_id = "base_link"
    out_msg.twist = msg

    ref_twiststamped_pub.publish(out_msg)


if __name__ == "__main__":
    rospy.init_node("scout_state_node")

    br = tf2_ros.TransformBroadcaster()
    # We subscribe the odometry topic (state)
    robot_state_sub = rospy.Subscriber("/Odometry", Odometry, jackal_msg_callback, queue_size=20)

    robot_state_pub = rospy.Publisher("/wild_visual_navigation_node/robot_state", RobotState, queue_size=20)

    # And also the twist command from teleoperation
    ref_twist_sub = rospy.Subscriber("/cmd_vel", Twist, twist_msg_callback, queue_size=20)
    ref_twiststamped_pub = rospy.Publisher("/wild_visual_navigation_node/reference_twist", TwistStamped, queue_size=20)

    rospy.loginfo("[scout_state_node] ready")
    rospy.spin()
