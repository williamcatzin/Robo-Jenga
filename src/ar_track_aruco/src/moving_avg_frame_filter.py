#!/usr/bin/env python

from collections import deque
from typing import Deque
import rospy
import tf2_py
import tf2_ros
import tf2_msgs.msg
import sys
import transformations
import helpers 

def main(args):
    parent_frame = args[1]
    child_frame = args[2]
    filtered_child_frame = args[3]
    frames_to_average = args[3]

    rospy.init_node('moving_avg_frame_filter')
    tfBuffer = tf2_ros.Buffer()
    _ = tf2_ros.TransformListener(tfBuffer)
    frame_pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)
    frame_deque = deque()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time())
            frame = helpers.tf_to_g(transform)
            frame_deque.appendleft(frame)
            if len(frame_deque) > frames_to_average:
                frame_deque.pop()
                sum_of_frames = sum(frame_deque)
                avg_frame = sum_of_frames / len(frame_deque)
                frame_pub.publish(helpers.g_to_tf(avg_frame, parent_frame, filtered_child_frame))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)