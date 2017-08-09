import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
import numpy as np

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
trans = TransformStamped()
bc = tf2_ros.TransformBroadcaster()

once = 1
while not rospy.is_shutdown():
    
    trans.header.frame_id = "world"
    trans.child_frame_id = "dist" 
    trans.header.stamp = rospy.Time.now()
    
    z_angle = np.pi/2
    y_angle = np.pi/2
    Rz = tf.transformations.rotation_matrix(z_angle, [0, 0, 1])
    Ry = tf.transformations.rotation_matrix(y_angle, [0, 1, 0])
    translation = tf.transformations.translation_matrix([1, 0, 0])
    ht = tf.transformations.concatenate_matrices(translation, Ry, Rz)
    if once == 1:
        print(np.round(ht,2))
        print(np.round(Rz,2))
        print(np.round(Ry,2))
        once += 1
    trans.transform = message_from_transform(ht)
    bc.sendTransform(trans)
    
    rate.sleep()