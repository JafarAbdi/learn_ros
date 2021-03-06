#!/usr/bin/env python
import rospy
import numpy as np
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import actionlib
from learn_ros.msg import GoToPointAction


class Go_To_Point:
    odom_topic = "/robot0/odom"
    twist_topic = "/robot0/cmd_vel"
    pt_topic = "/robot0/pt_d"  # Desired point

    th = 0
    pose = np.array([0., 0.])  # [x, y]

    def __init__(self):
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        self.TwistPub = rospy.Publisher(self.twist_topic, Twist, queue_size=10)
        self.ServerGTP = actionlib.SimpleActionServer(
            'go_to_point', GoToPointAction, self.pt_cb, False)
        self.ServerGTP.start()
        rospy.loginfo("Go To Point Controller Initialized")

    def odom_cb(self, msg):
        self.th = tf.transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        ])[-1]  # only z component

        self.pose = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y])

    def pt_cb(self, goal):
        self.go_to_point(goal.heading_pt.x, goal.heading_pt.y,
                         goal.heading_pt.theta)

    def go_to_point(self,
                    x,
                    y,
                    th,
                    th_kp=.8,
                    th_ki=.001,
                    th_kd=0.0,
                    d_kp=0.2,
                    eps=0.01):

        rospy.loginfo("Received New Desired Point x: %s, y: %s, theta: %s.", x,
                      y, th)
        pt_d = np.array([x, y])
        pose_to_d = pt_d - self.pose
        th_d = np.arctan2(pose_to_d[1], pose_to_d[0])
        TH_d = 0.
        last_e = 0.
        rate = rospy.Rate(100)
        twist_msg = Twist()
        while True and not rospy.is_shutdown():
            if self.ServerGTP.is_preempt_requested():
                self.ServerGTP.set_preempted()
                return
            pose_to_d = pt_d - self.pose

            th_d = np.arctan2(pose_to_d[1], pose_to_d[0])
            th_e = th_d - self.th
            th_e = np.arctan2(np.sin(th_e), np.cos(th_e))

            TH_d += th_e
            e_dd = th_e - last_e
            d = np.linalg.norm(pose_to_d)
            if d < eps:
                self.go_to_angle(th)
                break
            ang_vel = th_kp * th_e + th_kd * e_dd + th_ki * TH_d
            ang_vel = np.clip(ang_vel, -1.4, 1.4)
            lin_vel = d_kp * d
            lin_vel = np.clip(lin_vel, .05, 0.25)
            twist_msg.angular.z = ang_vel
            twist_msg.linear.x = lin_vel
            self.TwistPub.publish(twist_msg)
            rate.sleep()
            last_e = th_e
        twist_msg.angular.z = 0.
        twist_msg.linear.x = 0.
        self.TwistPub.publish(twist_msg)

    def go_to_angle(self, th_d, kp=1.5, eps=0.01):
        rate = rospy.Rate(100)
        twist_msg = Twist()
        while True and not rospy.is_shutdown():
            if self.ServerGTP.is_preempt_requested():
                self.ServerGTP.set_preempted()
                break
            th_e = th_d - self.th
            if abs(th_e) < eps:
                self.ServerGTP.set_succeeded()
                break
            twist_msg.angular.z = kp * th_e
            self.TwistPub.publish(twist_msg)
            rate.sleep()

        twist_msg.angular.z = 0.
        twist_msg.linear.x = 0.
        self.TwistPub.publish(twist_msg)


rospy.init_node("go_to_angle")
controller = Go_To_Point()
rospy.spin()
