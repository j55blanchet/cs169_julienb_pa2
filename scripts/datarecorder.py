#!/usr/bin/env python
"""
    datarecorder.py

    Author: Julien Blanchet
    Feb. 2 2020

    A node for for outputting state estimations to csv files
"""


import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class TOPIC:
    POSE = "pose"
    STATE_ESTIMATE = "state_estimate"

if __name__ == "__main__":

    rospy.init_node("datarecorder")

    record_topic = rospy.get_param("~recordtopic", default="pose")
    output_file = rospy.get_param("~outfile", default="data.csv")

    process_data = None
    msg_type = NotImplemented
    sub_topic = None

    if record_topic == TOPIC.POSE:
        process_data = lambda msg: str(msg.pose.position.x)
        msg_type = PoseStamped
        sub_topic = "pose"
    elif record_topic is TOPIC.STATE_ESTIMATE:
        process_data = lambda msg: str(msg.pose.pose.position.x)
        msg_type = PoseWithCovarianceStamped
        sub_topic = "state_estimate"
    else:
        rospy.logerr("Unrecognized record topic: " + str(record_topic))
        exit(1)

    with open(output_file, "w") as myfile:
        def data_received(msg):
            myfile.write(process_data(msg))
            myfile.write(",\n")

        rospy.Subscriber("pose", msg_type, data_received)
        rospy.spin()