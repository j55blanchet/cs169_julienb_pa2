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
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from kalmanfilter import KalmanFilter

INITIAL_BELIEF = numpy.matrix([0.0])
INITIAL_UNCERTAINTY = numpy.matrix([1.0])

STATE_EVOLUTION = numpy.matrix([1.0]) # We expect position to remain constant in absence of control

MOTION_NOISE = 0.05
SENSE_NOISE = 0.05

class MODE:
    CMD_VEL_SCAN = "b"
    POSE_SCAN = "c"
    CMD_VEL_RGBD = "d"
    POSE_RGBD = "e"
    
# class PARAM:
#     class PROPOGATION_SOURCE:
#         CMD_VEL = "cmd_vel"
#         POSE = "pose"
    
#     class UPDATE_SOURCE
#         SCAN = "scan"
#         CAMERA = "scan"

class PoseEstimator:

    def __init__(self):

        self.mode = rospy.get_param("~mode", default=MODE.CMD_VEL_RGBD)
        rospy.loginfo("Using mode: " + str(self.mode))

        if self.mode == MODE.CMD_VEL_SCAN or self.mode == MODE.CMD_VEL_RGBD:
            rospy.Subscriber("cmd_vel", Twist, self.on_cmdvel, queue_size=1)
        
        elif self.mode == MODE.POSE_SCAN or self.mode == MODE.POSE_RGBD:
            rospy.Subscriber("pose", PoseStamped, self.on_pose, queue_size=1)
        
        else:
            rospy.logerr("Unrecognized / invalid mode: " + self.mode)
            exit(1)

        rospy.Subscriber("scan", LaserScan, self.on_scan, queue_size=1)
        self.state_pub = rospy.Publisher("state_estimate", PoseWithCovarianceStamped, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        outfile = rospy.get_param("~outfile", default="output.csv")
        self.outfile = open(outfile, "w")
        self.outfile.write("time, location, uncertainty\n")

        self.kfilter = KalmanFilter(
            INITIAL_BELIEF,
            INITIAL_UNCERTAINTY,
            STATE_EVOLUTION,
            MOTION_NOISE,
            SENSE_NOISE)

        self.prev_cmdvel_t = None
        self.prev_cmdvel = None
        self.cmdvel_count = 0

        self.prev_pose = None
        self.pose_count = 0

        self.scan_count = 0
        self.prev_wall_loc = None
    
    def spin(self):
        rospy.spin()

    def on_cmdvel(self, msg):
        
        curtime = rospy.Time.now()

        self.cmdvel_count += 1
        rospy.loginfo("cmdvel #{0}     linX:{1}".format(self.cmdvel_count, msg.linear.x))

        if self.prev_cmdvel_t is not None:
            dt = curtime - self.prev_cmdvel_t
            avg_vel = 0.5 * (msg.linear.x + self.prev_cmdvel.linear.x)
            rospy.loginfo("\tavg_vel: {0}   dt:{1}".format(avg_vel, dt))
            
            self.kfilter.propogate(
                action_model=numpy.matrix([dt.to_sec()]),
                control=numpy.matrix([avg_vel])
            )
            self.publish_state_estimation(curtime)

        self.prev_cmdvel_t = curtime
        self.prev_cmdvel = msg
    
    def on_pose(self, msg):
        
        self.pose_count += 1
        rospy.loginfo("pose #{0}     location:{1}".format(self.pose_count, msg.pose.position.x))

        if self.prev_pose is not None:
            dx = msg.pose.position.x - self.prev_pose.pose.position.x
            rospy.loginfo("\tdx: {0}".format(dx))

            self.kfilter.propogate(
                action_model=numpy.matrix([1]),
                control=numpy.matrix([dx])
            )
            self.publish_state_estimation(msg.header.stamp)

        self.prev_pose = msg

    def on_scan(self, msg):
        self.scan_count += 1
        

        cur_reading = None        
        if self.mode == MODE.POSE_SCAN or self.mode == MODE.CMD_VEL_SCAN:
            # 0th range measurment is forward using the builtin lidar
            cur_reading = msg.ranges[0]
        elif self.mode == MODE.POSE_RGBD or self.mode == MODE.CMD_VEL_RGBD:
            # for RGBD conversion, the middle range measurement is forward
            #    > for increased robustness, look at the middle few messages
            tries = 0
            midI = int(len(msg.ranges) / 2)
            loffset = 0
            roffset = 0
            while cur_reading is None or math.isnan(cur_reading):
                if tries % 2 == 0:
                    cur_reading = msg.ranges[midI + loffset]
                    loffset -= 1
                if tries % 2 == 1:
                    cur_reading = msg.ranges[midI + roffset]
                    roffset += 1
                tries += 1
                if tries > 10:
                    break
            
        rospy.loginfo("scan #{0}     dist:{1}".format(self.scan_count, cur_reading))
        msg = LaserScan()
        
        if math.isnan(cur_reading):
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
            self.publish_state_estimation(msg.header.stamp)
 
        # Setting odom origin as 0, the wall is located at (scan_reading), offset by
        # our current location (stored in self.kfilter.belief)
        self.prev_wall_loc = self.kfilter.belief.item(0) + cur_reading
        rospy.loginfo("\twall location: {0}".format(self.prev_wall_loc))


    def publish_state_estimation(self, stamp):
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
                        stamp,
                        "odom_kf",
                        "base_footprint"
        )

        rospy.loginfo("\tkfilter. pos: {0}    uncertainty: {1}".format(self.kfilter.belief, self.kfilter.uncertainty))

        stamp = rospy.Time.now()

        self.outfile.write("{0},{1},{2}\n".format(stamp.to_sec(), robot_x, uncertainty))

    def __del__(self):
        self.outfile.close()
    

def main():
    rospy.init_node("pa2_poseestimator")

    pose_estimator = PoseEstimator()
    pose_estimator.spin()

if __name__ == "__main__":
    main()
