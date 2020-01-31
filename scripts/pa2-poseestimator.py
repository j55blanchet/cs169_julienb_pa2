#!/usr/bin/env python
"""
    pa2-poseestimator.py

    Author: Julien Blanchet
    Jan. 31 2020

    A basic impementation of a Kalman Filter, used for estimating the pose of a robot
"""

import rospy
import numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from kalmanfilter import KalmanFilter

INITIAL_BELIEF = numpy.matrix([0])
INITIAL_UNCERTAINTY = numpy.matrix([1])

STATE_EVOLUTION = numpy.matrix([1]) # We expect position to remain constant in absence of control
ACTION_MODEL = numpy.matrix([])

MOTION_NOISE = 0.05
SENSE_NOISE = 0.05

# class PARAM:
#     class PROPOGATION_SOURCE:
#         CMD_VEL = "cmd_vel"
#         POSE = "pose"
    
#     class UPDATE_SOURCE
#         SCAN = "scan"
#         CAMERA = "scan"

class PoseEstimator:

    def __init__(self):

        rospy.Subscriber("cmd_vel", Twist, self.on_cmdvel, queue_size=1)
        rospy.Subscriber("scan", LaserScan, self.on_scan, queue_size=1)
        state_pub = rospy.Publisher("state_estimate", PoseWithCovarianceStamped)
        
        self.kfilter = KalmanFilter(
            INITIAL_BELIEF,
            INITIAL_UNCERTAINTY,
            STATE_EVOLUTION,
            MOTION_NOISE,
            SENSE_NOISE)

        self.prev_cmdvel_t = None
        self.prev_cmdvel = None
        self.prev_scan = LaserScan()
    
    def spin(self):
        rospy.spin()

    def on_cmdvel(self, msg):
        
        curtime = rospy.Time.now()
        msg = Twist()

        if self.prev_cmdvel_t is not None and self.prev_cmdvel is not None:
            dt = curtime() - self.prev_cmdvel_t
            avg_vel = 0.5 * (msg.linear.x + self.prev_cmdvel.linear.x)
            
            self.kfilter.propogate(
                action_model=numpy.matrix([dt]),
                control=numpy.matrix([avg_vel])
            )
            pose = PoseWithCovarianceStamped()
            pose.pose.pose.position.x = self.kfilter.belief
            pose.pose.covariance = numpy.matrix(
                [self.kfilter.uncertainty, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                # Continue working here...

            )

        self.prev_cmdvel_t = curtime
        self.prev_cmdvel = msg
        

    def on_scan(self, msg):
        pass

def main():
    rospy.init_node("pa2_poseestimator")

    pose_estimator = PoseEstimator()
    pose_estimator.spin()

    # prop_sourcetopic = rospy.get_param("propogationtopic", default="cmd_vel")
    # update_sourcetopic = rospy.get_param("updatetopic", default="scan")


if __name__ == "__main__":
    main()
