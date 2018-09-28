#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
import numpy as np
from markdown.util import int2str

def message_from_transform(T):
    msg = Transform()
    q = tf.transformations.quaternion_from_matrix(T)
    p = tf.transformations.translation_from_matrix(T)
    msg.translation.x = p[0]
    msg.translation.y = p[1]
    msg.translation.z = p[2]
    msg.rotation.x = q[0]
    msg.rotation.y = q[1]
    msg.rotation.z = q[2]
    msg.rotation.w = q[3]
    return msg

rospy.init_node("trans")
rate = rospy.Rate(10)
bc = tf2_ros.TransformBroadcaster()

while not rospy.is_shutdown():
    
    transforms = []
    T1 = tf.transformations.rotation_matrix(np.pi/4, [0, 0, 1])
    T2 = tf.transformations.translation_matrix([0, 1, 0])
    T3 = tf.transformations.rotation_matrix(np.pi/2, [0, 0, 1])
    T4 = tf.transformations.rotation_matrix(np.pi/2, [0, 1, 0])
    T5 = tf.transformations.translation_matrix([1, 0, 0])
    #T6 = tf.transformations.rotation_matrix(np.pi/2, [0, 1, 0])
    
    for i in range(5):
         trans = TransformStamped()
         trans.header.frame_id = str(0)
         trans.child_frame_id = str(i + 1)
         trans.header.stamp = rospy.Time.now()
         transforms.append(trans)
    # the summary is when you want to rotate around the fixed frame you pre-multiply 
    # and when you want to rotate around the current frame you post-multiply
    T = T1
    transforms[0].transform = message_from_transform(T)
    T = np.dot(T, T2)
    transforms[1].transform = message_from_transform(T)
    T = np.dot(T3, T)
    transforms[2].transform = message_from_transform(T)
    T = np.dot(T4, T)
    transforms[3].transform = message_from_transform(T)
    T = np.dot(T, T5)
    transforms[4].transform = message_from_transform(T)
    
    bc.sendTransform(transforms)
    
    rate.sleep()