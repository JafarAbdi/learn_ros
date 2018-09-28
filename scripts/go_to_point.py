#!/usr/bin/env python
import rospy
import numpy as np
import tf.transformations as trans

from std_msgs.msg import Int32, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

class Go_To_Point:
    odom_topic  = "/robot0/odom"
    twist_topic = "/robot0/cmd_vel"
    pt_topic    = "/robot0/pt_d" # Desired point
    #vel_topic   = "/robot0/vel_d"
    #vel = 0 # linear velocity
    th  = 0
    pose= np.array([0., 0.]) # [x, y]
    def __init__(self):
        #rospy.Subscriber(  self.vel_topic,  Float32,   self.vel_cb)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        rospy.Subscriber(self.pt_topic  ,   Pose2D, self.pt_cb  )
        
        self.TwistPub = rospy.Publisher(self.twist_topic, Twist, queue_size=10)
        
        rospy.loginfo("Go To Point Controller Initialized")
    #def vel_cb(self, msg):
    #    self.vel = msg.data
    def odom_cb(self, msg):
        self.th = trans.euler_from_quaternion([msg.pose.pose.orientation.x, 
                                               msg.pose.pose.orientation.y, 
                                               msg.pose.pose.orientation.z, 
                                               msg.pose.pose.orientation.w])[-1] # z
                    
        self.pose = np.array([msg.pose.pose.position.x, 
                              msg.pose.pose.position.y])                           
    def pt_cb(self, msg):
        self.go_to_point(msg.x, msg.y)
        self.go_to_angle(msg.theta)
    def go_to_point(self, x, y, th_kp=.8, th_ki=.001, th_kd=0.0, d_kp = 0.2, eps=0.01):
        pt_d = np.array([x,
                         y])
        pose_to_d = pt_d - self.pose
        th_d = np.arctan2(pose_to_d[1], pose_to_d[0])
        TH_d = 0.
        last_e = 0.
        rate = rospy.Rate(100)
        twist_msg = Twist()
        while True:
            pose_to_d = pt_d - self.pose
            
            th_d = np.arctan2(pose_to_d[1], pose_to_d[0])
            th_e = th_d - self.th
            th_e = np.arctan2(np.sin(th_e), np.cos(th_e))
            
            TH_d += th_e
            e_dd = th_e - last_e
            d    = np.linalg.norm(pose_to_d)
            if d < eps:
                break 
            ang_vel = th_kp * th_e + th_kd * e_dd + th_ki * TH_d
            ang_vel = np.clip(ang_vel, -1.4, 1.4)
            lin_vel = d_kp * d
            lin_vel = np.clip(lin_vel, .025, 0.25)
            print(th_e, e_dd, TH_d, lin_vel)
            twist_msg.angular.z = ang_vel
            twist_msg.linear.x  = lin_vel
            self.TwistPub.publish(twist_msg)
            rate.sleep()
            last_e = th_e
        twist_msg.angular.z = 0.
        twist_msg.linear.x  = 0.
        self.TwistPub.publish(twist_msg)
    
    def go_to_angle(self, th_d, kp=1.5, eps=0.01):
        rospy.loginfo("Received New Desired Angle %d", th_d)
        rate = rospy.Rate(100)
        twist_msg = Twist()
        while True:
            th_e = th_d - self.th
            if abs(th_e) < eps:
                break 
            twist_msg.angular.z = kp * th_e
            #twist_msg.linear.x  = self.vel
            self.TwistPub.publish(twist_msg)
            rate.sleep()
            
        twist_msg.angular.z = 0.
        twist_msg.linear.x  = 0.
        self.TwistPub.publish(twist_msg)
        print(self.pose[0], self.pose[1], self.th)
        
rospy.init_node("go_to_angle")
controller = Go_To_Point()
rospy.spin()

    