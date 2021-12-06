#!/usr/bin/env python

import sys

from baxter_interface import Limb

import helpers 
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
# from geographic_msgs import Transform
# from geographic_msgs import Vector3
# from geographic_msgs import Quaternion

from path_planner import PathPlanner
try:
    from controller import Controller
except ImportError:
    pass

import transformations

import tf2_ros

import tf2_msgs.msg
import geometry_msgs.msg

def g_to_vec(g):
    trans = transformations.translation_from_matrix(g)
    rot = transformations.quaternion_from_matrix(g)
    return trans, rot

def vec_to_g(trans, rot):
    return np.matmul(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))

def g_to_tf(g, parent_frame_id, child_frame_id):

    trans, rot = g_to_vec(g)

    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent_frame_id
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform.translation.x = trans[0]
    t.transform.translation.y = trans[1]
    t.transform.translation.z = trans[2]

    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]
    
    tfm = tf2_msgs.msg.TFMessage([t])
    
    return tfm

def tf_to_g(transform):
    trans = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
    rot = np.array([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
    return vec_to_g(trans, rot)
    

def g_to_pose(g, parent_frame):
    trans, rot = g_to_vec(g)

    pose = PoseStamped()
    pose.header.frame_id = parent_frame

    #x, y, and z position
    pose.pose.position.x = trans[0]
    pose.pose.position.y = trans[1]
    pose.pose.position.z = trans[2]

    #Orientation as a quaternion
    pose.pose.orientation.x = rot[0]
    pose.pose.orientation.y = rot[1]
    pose.pose.orientation.z = rot[2]
    pose.pose.orientation.w = rot[3]

    return pose
    