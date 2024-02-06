#!/usr/bin/env python3
import rospy
from heightmap_navigation.msg import HeightMap
import numpy as np 
from cv_bridge import CvBridge

if __name__ == '__main__':
    rospy.init_node('testmap_publisher', anonymous=True)
    pub = rospy.Publisher('height_map', HeightMap, queue_size=1)
    bridge = CvBridge()
    for i in range(30):
        map_msg = HeightMap()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "robot_map"
        if False:
            map_msg.resolution_x = 0.2
            map_msg.resolution_y = 0.2
            height_map_img = np.zeros((32,32), dtype=np.float32)
            height_map_img[0,:] = 0.5
            height_map_img[-1,:] = 0.5
            height_map_img[:, 0] = 0.5
            height_map_img[:, -1] = 0.5
            height_map_img[6:26, 19:22] = -0.2
            height_map_img[6:9, 6:26] = -0.2
        else :
            map_msg.resolution_x = 0.05
            map_msg.resolution_y = 0.05

            # height_map_img = np.zeros((128,128), dtype=np.float32) # 6.4m四方
            # height_map_img[0,:] = 0.5
            # height_map_img[-1,:] = 0.5
            # height_map_img[:, 0] = 0.5
            # height_map_img[:, -1] = 0.5
            # height_map_img[30:90, 80:90] = -0.2
            # height_map_img[30:40, 30:90] = -0.2

            
            height_map_img = np.zeros((64,64), dtype=np.float32) # 6.4m四方
            height_map_img[0,:] = 0.5
            height_map_img[-1,:] = 0.5
            height_map_img[:, 0] = 0.5
            height_map_img[:, -1] = 0.5
            height_map_img[20:40, 42:45] = -0.2
            height_map_img[18:20, 20:40] = -0.2
        map_msg.map = bridge.cv2_to_imgmsg(height_map_img, encoding="passthrough")
        pub.publish(map_msg)
        rospy.sleep(0.1)

