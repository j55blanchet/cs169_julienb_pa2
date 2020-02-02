#!/usr/bin/env python
"""
    pa2-poseestimator.py

    Author: Julien Blanchet
    Jan. 31 2020

    A basic impementation of a Kalman Filter, used for estimating the pose of a robot
"""

import rospy
import numpy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from kalmanfilter import KalmanFilter

INITIAL_BELIEF = numpy.matrix([0.0])
INITIAL_UNCERTAINTY = numpy.matrix([1.0])

STATE_EVOLUTION = numpy.matrix([1.0]) # We expect position to remain constant in absence of control

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
        self.state_pub = rospy.Publisher("state_estimate", PoseWithCovarianceStamped, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        outfile = rospy.get_param("~outfile", default="output.csv")
        self.outfile = open(outfile, "w")

        self.kfilter = KalmanFilter(
            INITIAL_BELIEF,
            INITIAL_UNCERTAINTY,
            STATE_EVOLUTION,
            MOTION_NOISE,
            SENSE_NOISE)

        self.prev_cmdvel_t = None
        self.prev_cmdvel = None
        self.cmdvel_count = 0
        self.scan_count = 0
        self.prev_wall_loc = None
    
    def spin(self):
        rospy.spin()

    def on_cmdvel(self, msg):
        
        curtime = rospy.Time.now()

        self.cmdvel_count += 1
        rospy.loginfo("Received cmdvel msg #{0} linX:{1}".format(self.cmdvel_count, msg.linear.x))

        if self.prev_cmdvel_t is not None and self.prev_cmdvel is not None:
            dt = curtime - self.prev_cmdvel_t
            avg_vel = 0.5 * (msg.linear.x + self.prev_cmdvel.linear.x)
            rospy.loginfo("\tavg_vel: {0}   dt:{1}".format(avg_vel, dt))
            
            self.kfilter.propogate(
                action_model=numpy.matrix([dt.to_sec()]),
                control=numpy.matrix([avg_vel])
            )
            self.publish_state_estimation()

        self.prev_cmdvel_t = curtime
        self.prev_cmdvel = msg
        

    def on_scan(self, msg):
        self.scan_count += 1

        cur_reading = msg.ranges[0]
        rospy.loginfo("Received scan msg #{0} dist:{1}".format(self.scan_count, cur_reading))
        
        if cur_reading > msg.range_max or cur_reading < msg.range_min:
            rospy.loginfo("\tUnusable scan reading. Ignoring scan msg")
            return

        if self.prev_wall_loc is not None:

            # Need to jump through some hoops to get sensor model,
            # because we choose to make 0 be the starting point of the robot
            robot_x = self.kfilter.belief.item(0)
            expected_wall_distance = self.prev_wall_loc - robot_x

            self.kfilter.update(
                expected_reading=expected_wall_distance,
                reading=cur_reading
            )
            self.publish_state_estimation()
 
        # Setting odom origin as 0, the wall is located at (scan_reading), offset by
        # our current location (stored in self.kfilter.belief)
        self.prev_wall_loc = self.kfilter.belief.item(0) + cur_reading
        rospy.loginfo("\testimated wall location: {0}".format(self.prev_wall_loc))


    def publish_state_estimation(self):
        robot_x = self.kfilter.belief.item(0)
        uncertainty = self.kfilter.uncertainty.item(0)
        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position.x = robot_x
        pose.pose.covariance = numpy.matrix([
            [uncertainty, 0, 0, 0, 0, 0],
            [0,           0, 0, 0, 0, 0],
            [0,           0, 0, 0, 0, 0],
            [0,           0, 0, 0, 0, 0],
            [0,           0, 0, 0, 0, 0],
            [0,           0, 0, 0, 0, 0]
        ])
        self.state_pub.publish(pose)
        
        self.tf_broadcaster.sendTransform(
                        (robot_x, 0, 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "odom_kf",
                        "base_footprint"
        )

        rospy.loginfo("\tposition: {0}    uncertainty: {1}".format(self.kfilter.belief, self.kfilter.uncertainty))

        self.outfile.write("{0},{1}\n".format(robot_x, uncertainty))
        rospy.loginfo("\twrote data to file at {0}".format(self.outfile.name))

    def __del__(self):
        self.outfile.close()
    

def main():
    rospy.init_node("pa2_poseestimator")

    pose_estimator = PoseEstimator()
    pose_estimator.spin()

    # prop_sourcetopic = rospy.get_param("propogationtopic", default="cmd_vel")
    # update_sourcetopic = rospy.get_param("updatetopic", default="scan")


if __name__ == "__main__":
    main()
