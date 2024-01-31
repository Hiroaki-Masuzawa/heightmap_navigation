#!/usr/bin/env python3
import rospy
from heightmap_navigation.msg import HeightMap
from nav_msgs.msg import OccupancyGrid
import numpy as np 
from cv_bridge import CvBridge

class HeightMap2OccupancyGrid:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('map', OccupancyGrid, queue_size=1)
        self.sub = rospy.Subscriber("height_map", HeightMap, self.callback)
        self.min_height = -0.02
        self.max_height = 0.02

    def callback(self, map_msg):
        map_data = self.bridge.imgmsg_to_cv2(map_msg.map, desired_encoding='passthrough')
        occ_map = OccupancyGrid()
        occ_map.header = map_msg.header
        occ_map.info.map_load_time = map_msg.header.stamp
        occ_map.info.resolution = map_msg.resolution_x # TODO if map_msg.resolution_x != map_msg.resolution_y
        occ_map.info.width = map_msg.map.width
        occ_map.info.height = map_msg.map.height
        occ_map.info.origin.position.x = (-map_msg.map.width-0.5)*map_msg.resolution_x/2
        occ_map.info.origin.position.y = (-map_msg.map.height-0.5)*map_msg.resolution_y/2
        occ_map.info.origin.position.z = 0
        occ_map.info.origin.orientation.x = 0
        occ_map.info.origin.orientation.y = 0
        occ_map.info.origin.orientation.z = 0
        occ_map.info.origin.orientation.w = 1
        for py in range(map_msg.map.height):
            for px in range(map_msg.map.width):
                occ_map.data.append(100 if self.min_height <= map_data[py,px] <= self.max_height else 0)
        self.pub.publish(occ_map)
        
if __name__ == '__main__':
    rospy.init_node('height2occgrid', anonymous=True)
    node_ins = HeightMap2OccupancyGrid()
    rospy.spin()
    
