#!/usr/bin/env python
"""
    datarecorder.py

    Author: Julien Blanchet
    Feb. 2 2020

    A node for for outputting state estimations to csv files
"""

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

if __name__ == "__main__":

    rospy.init_node("datarecorder")

    output_file = rospy.get_param("~outfile", default="data.csv")

    process_data = lambda msg: "{0},{1}".format(rospy.Time.now().to_sec(), msg.pose.position.x)
    msg_type = PoseStamped
    sub_topic = "pose"

    with open(output_file, "w") as myfile:

        myfile.write("time, position\n")

        def data_received(msg):
            myfile.write(process_data(msg))
            myfile.write("\n")

        rospy.Subscriber("pose", msg_type, data_received)
        rospy.spin()