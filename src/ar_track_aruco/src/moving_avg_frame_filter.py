#!/usr/bin/env python

from collections import deque
import rospy
import tf2_py
import tf2_ros
import tf2_msgs.msg
import sys
import transformations
import helpers 
import numpy as np

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
    direction_deque = deque()
    angle_deque = deque()
    trans_deque = deque()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            #print(len(frame_deque))
            transform = tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time())
            #print(transform)
            angle, direction, _ = transformations.rotation_from_matrix(helpers.tf_to_g(transform))
            trans = transformations.translation_from_matrix(helpers.tf_to_g(transform))
            angle_deque.appendleft(angle)
            direction_deque.appendleft(direction)
            trans_deque.appendleft(trans)
            if len(direction_deque) > frames_to_average:
                direction_deque.pop()
                angle_deque.pop()
                trans_deque.pop()
                sum_of_directions = sum(direction_deque)
                avg_direction = sum_of_directions / frames_to_average
                sum_of_angles = sum(angle_deque)
                avg_angle = sum_of_angles / frames_to_average
                sum_of_trans = sum(trans_deque)
                avg_trans = sum_of_trans / frames_to_average

                avg_rot_mat = transformations.rotation_matrix(avg_angle, avg_direction)
                avg_trans_mat = transformations.translation_matrix(avg_trans)

                avg_frame = np.matmul(avg_trans_mat, avg_rot_mat)
                
                frame_pub.publish(helpers.g_to_tf(avg_frame, parent_frame, filtered_child_frame))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)