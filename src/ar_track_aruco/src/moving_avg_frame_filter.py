#!/usr/bin/env python

from collections import deque
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
    frames_to_average = int(args[4])

    print("parent frame: {}".format(parent_frame))
    print("child frame: {}".format(child_frame))
    print("filtered child frame: {}".format(filtered_child_frame))
    print("frames to average: {}".format(frames_to_average))

    rospy.init_node('moving_avg_frame_filter', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    _ = tf2_ros.TransformListener(tfBuffer)
    frame_pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)
    frame_deque = deque()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            #print(len(frame_deque))
            transform = tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time())
            #print(transform)
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