#!/usr/bin/env python3
import math
import numpy as np 
import copy
import cv2
from enum import IntEnum

# ROS関係
import rospy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, Quaternion 
import tf
import tf2_ros

from heightmap_navigation.msg import HeightMap

# Network solver
import networkx as nx

import scipy.interpolate as si

class GridMap:
    def __init__(self):
        self.MAP = nx.Graph()   # 空のグラフ生成
        self.costmap = None
        self.num_grid_x = 0
        self.num_grid_y = 0
    def setCostMap(self, costmap):
        """ コストマップの設定 """
        self.costmap = costmap
        self.MAP = nx.grid_2d_graph(costmap.shape[1], costmap.shape[0])
        self.num_grid_x = costmap.shape[1]
        self.num_grid_y = costmap.shape[0]
        self.addCrossEdge(self.MAP, 1)
        nx.set_edge_attributes(self.MAP, {e: costmap[e[1][1], e[1][0]] for e in self.MAP.edges()}, "cost")
        # +0.05*math.sqrt((e[0][0]-e[1][0])**2+(e[1][0]-e[1][1])**2)
    def getMap(self):
        """ マップを返す """
        return self.MAP
    def addCrossEdge(self, gp, step):
        """ 2DGridのグラフに斜め方向のエッジを追加する """

        x, y = np.array(gp.nodes()).T
        p_min = (min(x), min(y))
        p_max = (max(x), max(y))

        for node in list(gp.nodes()):
            nx_node = (node[0] + step, node[1] + step)
            if nx_node[0] <= p_max[0] and nx_node[1] <= p_max[1]:
                gp.add_edge(node, nx_node)
            nx_node = (node[0] + step, node[1] - step)
            if nx_node[0] <= p_max[0] and nx_node[1] >= p_min[1]:
                gp.add_edge(node, nx_node)


class NodeStatus(IntEnum):
    UNUSE = 0
    USE = 1
    SMOOTH_START = 10
    INTER_SMOOTH = 2
    SMOOTH_END = 20
# class SearchPath(tf_handler.TfHandler):

class SearchPath():
    """ ルート生成クラス """

    # def __init__(self, MAP, dest):
    #     tf_handler.TfHandler.__init__(self)

    #     self.MAP = MAP      # map
    #     self.dest = dest    # 目的地
    #     self.nodes_pos = dict((n, n) for n in self.MAP.nodes())     # 各nodeの座標
    

    def __init__(self):
        self.resolutionX = 0
        self.resolutionY = 0
        self.target_map = None

    def setResolution(self, resolutionX, resolutionY):
        self.resolutionX = resolutionX
        self.resolutionY = resolutionY
    def setTargetMap(self, target_map):
        self.target_map = target_map

    def generate_path(self, start, goal):
        """ A*でルート生成 """

        g = self.dest[goal]     # ゴールの座標

        # スタートの座標
        if start == 'local':    # 'local'なら現在座標
            p, _  = self.get_tf_pos('map', 'base_link')
            theta = np.pi / 2
            rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
            p = np.dot(rot, p[0:2])
            s = tuple(np.round(p*20).astype(int)*5)
        else:
            s = self.dest[start]

        return nx.astar_path(self.MAP, s, g, heuristic=self.dist, weight='weight')

    def generate_smoothly_path(self, start, goal, degree=3):
        """ B-splineでルート生成 """

        path = self.generate_path(start, goal)

        return self.bspline(path, n=100, degree=degree, periodic=False)

    def dist(self, a, b):
        """ 2点間の距離 """
        (x1, y1) = a
        (x2, y2) = b
        dist = ((self.resolutionX*(x1 - x2)) ** 2 + (self.resolutionY*(y1 - y2)) ** 2) ** 0.5
        # print(a,b,dist)
        return dist

    def bspline(self, cv, n=100, degree=3, periodic=False):
        """ B-splineのnサンプルを計算する

            cv :      制御点の配列
            n  :      サンプル数
            degree:   曲線の次数
            periodic: True - 閉じている, False - 開いている
        """

        # 周期的な場合, 制御点の配列を count+degree+1 だけ拡張する
        cv = np.asarray(cv)
        count = len(cv)

        if periodic:
            factor, fraction = divmod(count+degree+1, count)
            cv = np.concatenate((cv,) * factor + (cv[:fraction],))
            count = len(cv)
            degree = np.clip(degree,1,degree)

        # 開いている場合, 次数が count-1 を超えないようにする
        else:
            degree = np.clip(degree,1,count-1)


        # ノットベクトルを計算
        kv = None
        if periodic:
            kv = np.arange(0-degree,count+degree+degree-1)
        else:
            kv = np.clip(np.arange(count+degree+1)-degree,0,count-degree)

        # クエリ範囲を計算
        u = np.linspace(periodic,(count-degree),n)

        # 結果を計算
        return np.array(si.splev(u, (kv,cv.T,degree))).T

    def get_grid_idx(self, pos):
        num_grid_x = self.target_map.num_grid_x
        num_grid_y = self.target_map.num_grid_y
        min_x = -num_grid_x/2*self.resolutionX
        min_y = -num_grid_y/2*self.resolutionY
        x_idx = int(round((pos[0]-min_x)/self.resolutionX))
        y_idx = int(round((pos[1]-min_y)/self.resolutionY))
        return (x_idx, y_idx)

    def get_real_pos(self, idx):
        num_grid_x = self.target_map.num_grid_x
        num_grid_y = self.target_map.num_grid_y
        x = (idx[0] - num_grid_x/2)*self.resolutionX
        y = (idx[1] - num_grid_y/2)*self.resolutionY
        return (x, y)

    def generate_path_from_coordinate(self, start, goal):
        
        # Astarで経路を生成する
        start_pos = self.get_grid_idx(start)
        goal_pos = self.get_grid_idx(goal)
        ret_idxs = nx.astar_path(self.target_map.getMap(), start_pos, goal_pos, heuristic=self.dist, weight="cost")
        

        node_status = np.ones((len(ret_idxs),), dtype=np.int32)

        ## 前後で変化しない点は削除候補とする
        for i in range(1, len(ret_idxs)-1):
            v1 = np.array(ret_idxs[i])-np.array(ret_idxs[i-1])
            v2 = np.array(ret_idxs[i+1])-np.array(ret_idxs[i])
            if (v2-v1 == np.zeros_like(v2-v1)).all():
                node_status[i] = NodeStatus.UNUSE

        ## 変化点のplus minus Nの点は補完のために残す
        node_status_tmp = copy.copy(node_status)
        N = 8
        for i in range(1, len(ret_idxs)-1):
            if node_status[i] ==1:
                for j in range(max(0,i-N), min(i+N+1, len(ret_idxs))):
                    if j == max(0,i-N):
                        node_status_tmp[j] = NodeStatus.SMOOTH_START
                    elif j == min(i+N+1, len(ret_idxs))-1:
                        node_status_tmp[j] = NodeStatus.SMOOTH_END
                    else :
                        node_status_tmp[j] = NodeStatus.INTER_SMOOTH
        node_status = node_status_tmp

        ret_idxs = np.array(ret_idxs)[node_status!=0, :]
        node_status = node_status[node_status!=0]
        start_idx = -1
        end_idx = -1
        ret_path = [self.get_real_pos(idx) for idx in ret_idxs]
        smooth_path = [ret_path[0]] if node_status[0] != NodeStatus.SMOOTH_START else []

        # print(node_status)
        for i in range(len(node_status)):
            if node_status[i] == NodeStatus.SMOOTH_START and start_idx==-1:
                start_idx = i
            elif node_status[i] == NodeStatus.SMOOTH_END:
                end_idx = i
                spline_path = self.bspline(ret_path[start_idx:end_idx+1], degree=5)
                # print("debug spline", ret_path[start_idx:end_idx+1], spline_path)
                start_idx = -1
                end_idx = -1
                for p in spline_path:
                    smooth_path.append(p)
        if node_status[-1] == NodeStatus.USE:
            smooth_path.append(ret_path[-1])
        return smooth_path, ret_path


class makePath:
    def __init__(self):
        # heightmapからgridmapを生成するパラメタ
        self.min_height = rospy.get_param('~min_height', -0.02)
        self.max_height = rospy.get_param('~max_height', 0.02)
        self.robot_frame_id = rospy.get_param('~robot_frame', 'real/base_link')
        self.map_frame_id = rospy.get_param('~map_frame', 'robot_map')
        self.bridge = CvBridge()
        self.map = GridMap()
        self.map_exist = False
        self.searchpath = SearchPath()
        
        # tflistener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # publisherの登録
        self.path_pub = rospy.Publisher("path", Path, queue_size=1)
        self.smooth_path_pub = rospy.Publisher("smooth_path", Path, queue_size=1)
        self.costmap_pub = rospy.Publisher('costmap', OccupancyGrid, queue_size=1)
        
        # subscriberの登録
        self.map_sub = rospy.Subscriber("height_map", HeightMap, self.callback_map)
        self.pos_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback_pose)

    def callback_map(self, map_msg):
        raw_map_data = self.bridge.imgmsg_to_cv2(map_msg.map, desired_encoding='passthrough')
        binarymap = np.ones_like(raw_map_data, dtype=np.uint8)
        binarymap[raw_map_data>=self.max_height] = 0
        binarymap[raw_map_data<=self.min_height] = 0
        dist_from_obj = cv2.distanceTransform(binarymap, cv2.DIST_L2, 5)
        inflation = 5 #10
        k_cost = 20
        costmap = np.clip(np.exp(-k_cost*np.clip((dist_from_obj-inflation), 0, 1e5)), 0, 1)*100

        costmap_view = OccupancyGrid()
        costmap_view.header = map_msg.header
        costmap_view.info.map_load_time = map_msg.header.stamp
        costmap_view.info.resolution = map_msg.resolution_x # TODO if map_msg.resolution_x != map_msg.resolution_y
        costmap_view.info.width = map_msg.map.width
        costmap_view.info.height = map_msg.map.height
        costmap_view.info.origin.position.x = (-map_msg.map.width-0.5)*map_msg.resolution_x/2
        costmap_view.info.origin.position.y = (-map_msg.map.height-0.5)*map_msg.resolution_y/2
        costmap_view.info.origin.position.z = 0
        costmap_view.info.origin.orientation = Quaternion(0,0,0,1)
        for py in range(map_msg.map.height):
            for px in range(map_msg.map.width):
                costmap_view.data.append(int(costmap[py][px]))
        self.costmap_pub.publish(costmap_view)

        self.map.setCostMap(costmap)
        self.searchpath.setTargetMap(self.map)
        self.searchpath.setResolution(map_msg.resolution_x, map_msg.resolution_y)
        self.map_exist = True

    def callback_pose(self, goal_pose):
        if not self.map_exist:
            print("map is not recieved yet")
            return 
        rospy.sleep(0.1)
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


        ret_path2, ret_path = self.searchpath.generate_path_from_coordinate((pos[0], pos[1]), (goal_pose.pose.position.x, goal_pose.pose.position.y))

        gen_path = Path()
        gen_path.header = goal_pose.header
        gen_path.header.stamp = rospy.Time.now()
        for p in ret_path:
            pose = PoseStamped()
            pose.pose.position = Point(p[0], p[1], 0.)
            pose.pose.orientation = Quaternion(0,0,0,1)
            gen_path.poses.append(pose)
        self.path_pub.publish(gen_path)


        gen_path = Path()
        gen_path.header = goal_pose.header
        gen_path.header.stamp = rospy.Time.now()
        for p in ret_path2:
            pose = PoseStamped()
            pose.pose.position = Point(p[0], p[1], 0.)
            pose.pose.orientation = Quaternion(0,0,0,1)
            gen_path.poses.append(pose)
        self.smooth_path_pub.publish(gen_path)

if __name__ == '__main__':
    rospy.init_node('make_path', anonymous=True)
    mp = makePath()
    rospy.spin()