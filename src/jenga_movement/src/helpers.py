def tf_publish(g, parent_frame_id, child_frame_id):

    rot,trans = g_to_vec(g)

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
    frame_pub.publish(tfm)

def g_to_vec(g):
    trans = transformations.translation_from_matrix(g)
    rot = transformations.quaternion_from_matrix(g)
    return rot, trans
