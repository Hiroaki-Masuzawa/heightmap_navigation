#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math

import numpy as np
import rospy
import tf2_ros
import tf

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist

class TracePath:
    def __init__(self):
        self.is_trace = False
        self.trace_path = None
        self.pos_route = []
        self.robot_frame_id = rospy.get_param('~robot_frame', 'real/base_link')
        self.map_frame_id = rospy.get_param('~map_frame', 'robot_map')
        self.goal_th = rospy.get_param('~goal_th', 0.2)
        
        # tflistener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # publisehr
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # subscriber
        self.path_sub = rospy.Subscriber("path", Path, self.callback_path)
        # timer
        self.timer = rospy.Timer(rospy.Duration(1), self.callback_timer)

    def callback_path(self, path_msg):
        self.path_msg = path_msg
        self.pos_route = []
        for pose in path_msg.poses:
            self.pos_route.append(np.array([pose.pose.position.x, pose.pose.position.y]))
        self.is_trace = True
        self.num_route = 0 
        
    def callback_timer(self, event):
        if not self.is_trace:
            return 
        stamp = rospy.Time.now() 
        rospy.sleep(0.1)
        try:
            t = self.tfBuffer.lookup_transform(self.map_frame_id , self.robot_frame_id, stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            return
        else:
            pos = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ])
            euler = np.array(
                tf.transformations.euler_from_quaternion((
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w
                ))
            )
        while np.linalg.norm(self.pos_route[self.num_route] - pos[0:2]) < self.goal_th:
            if self.num_route >= len(self.pos_route)-1:
                self.cmd_vel_pub.publish()
                self.is_trace = False
                break
            self.num_route += 1
        
        print("{}/{}".format(self.num_route, len(self.pos_route)))
        if self.is_trace:
            cmd_vel = Twist()
            if False :
                diff = self.pos_route[self.num_route] - pos[0:2]
                l = np.linalg.norm(diff)
                cmd_vel.linear.x = 0.1 * diff[0]/l
                cmd_vel.linear.y = 0.1 * diff[1]/l
                cmd_vel.angular.z = 0.0
            else :
                diff = self.pos_route[self.num_route] - pos[0:2]
                l = np.linalg.norm(diff)
                cos_theta = diff[0]/l
                sin_theta = diff[1]/l
                theta = math.atan2(diff[1], diff[0])
                diff_theta = theta-euler[2]
                v_const = 0.1
                while diff_theta >= math.pi:
                    diff_theta -= 2*math.pi
                while diff_theta < -math.pi:
                    diff_theta += 2*math.pi
                print(diff, theta, euler[2], diff_theta)
                if abs(diff_theta) > 0.4:
                    cmd_vel.angular.z = 0.25 if diff_theta > 0 else -0.25
                else :
                    cmd_vel.linear.x = v_const
                    cmd_vel.linear.y = 0.0
                    cmd_vel.angular.z = diff_theta/(l/v_const)

            self.cmd_vel_pub.publish(cmd_vel)

    

if __name__ == '__main__':
    rospy.init_node('trace_path', anonymous=True)
    tp = TracePath()
    rospy.spin()
