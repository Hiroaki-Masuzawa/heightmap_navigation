#!/usr/bin/env python3
import rospy
from heightmap_navigation.msg import HeightMap
import numpy as np 
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose, Vector3

class HeightMapVisualizer:
    def __init__(self):
        self.pub = rospy.Publisher('height_map_marker', MarkerArray, queue_size=1)
        self.bridge = CvBridge()
        self.z_offset = 0.05
        
        self.sub = rospy.Subscriber("height_map", HeightMap, self.callback)
        

    def callback(self, map_msg):
        marker_msg = MarkerArray()
        map_data = self.bridge.imgmsg_to_cv2(map_msg.map, desired_encoding='passthrough')
        height_min = np.min(map_data)
        for py in range(map_msg.map.height):
            for px in range(map_msg.map.width):
                x = (px-map_msg.map.width/2)*map_msg.resolution_x
                y = (py-map_msg.map.height/2)*map_msg.resolution_y
                box_height = map_data[py,px]-height_min+self.z_offset
                z = box_height/2+height_min-self.z_offset
                marker = Marker()
                marker.header=map_msg.header
                marker.ns = "heightmap"
                marker.id = py * map_msg.map.width + px
                marker.type = Marker.CUBE
                marker.action = 0
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z
                pose.orientation.x = 0
                pose.orientation.y = 0
                pose.orientation.z = 0
                pose.orientation.w = 1
                marker.pose = pose
                marker.scale.x = map_msg.resolution_x
                marker.scale.y = map_msg.resolution_y
                marker.scale.z = box_height
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.5
                marker_msg.markers.append(marker)
        self.pub.publish(marker_msg)
                


if __name__ == '__main__':
    rospy.init_node('height_map_visualizer', anonymous=True)
    node_ins = HeightMapVisualizer()
    rospy.spin()
    
