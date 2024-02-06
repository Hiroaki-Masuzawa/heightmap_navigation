#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import numpy as np
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist

class TracePath:
    def __init__(self):
        self.is_trace = False
        self.trace_path = None
        self.pos_route = []
        self.robot_frame_id = rospy.get_param('~robot_frame', 'base_link')
        self.map_frame_id = rospy.get_param('~map_frame', 'map')
        self.goal_th = 0.1
        
        # tflistener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # publisehr
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # subscriber
        self.path_sub = rospy.Subscriber("path", Path, self.callback_path)
        # timer
        self.timer = rospy.Timer(rospy.Duration(2), self.callback_timer)

    def callback_path(self, path_msg):
        self.path_msg = path_msg
        for pose in path_msg.poses:
            self.pos_route.append(np.array([pose.position.x, pose.position.y]))
        self.is_trace = True
        self.num_route = 0 
        
    def callback_timer(self, event):
        if not self.is_trace:
            return 
        
        try:
            t = self.tfBuffer.lookup_transform(self.map_frame_id , self.robot_frame_id, goal_pose.header.stamp)
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
        while np.linalg.norm(self.pos_route[self.num_route] - p[0:2]) < self.goal_th:
            if self.num_route >= self.pos_route.shape[0]-1:
                self.publish()
                self.is_trace = False
                break
            self.num_route += 1
        
        cmd_vel = Twist()
        diff = self.pos_route[self.num_route] - p[0:2]
        l = np.linalg.norm(diff)
        cmd_vel.linear.x = 0.1 * diff[0]/l
        cmd_vel.linear.y = 0.1 * diff[1]/l
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    

if __name__ == '__main__':
    rospy.init_node('trace_path', anonymous=True)
    tp = TracePath()
    rospy.spin()
