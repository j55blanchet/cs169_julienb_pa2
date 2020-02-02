#!/usr/bin/env python
"""
    odomtf_publisher.py

    Author: Julien Blanchet
    Feb. 2 2020

    A simple node that listens to /initialpose and publishes the appropiate transform 
    between the world and odom_kf frames
"""

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

def main():

    br = tf.TransformBroadcaster()

    def on_initialpose_received(msg):
        msg = PoseWithCovarianceStamped()

        position = msg.pose.pose.position

        br.sendTransform(
            (position.x, position.y, position.z),
            msg.pose.pose.orientation,
            msg.header.stamp,
            "odom_kf",
            "base_footprint"
        )

    rospy.Subscriber("initialpose", PoseWithCovarianceStamped, on_initialpose_received, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("odomtf_publisher")
    main()