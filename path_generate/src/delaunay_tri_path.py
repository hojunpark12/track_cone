#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,PoseStamped
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path

import matplotlib.tri as mtri
import numpy as np
import math
import copy

class Node:
    def __init__(self, x : float, y : float, color = None):
        self.x = x                              
        self.y = y            
                          
        if(color == 0 or color == "B" or color == "blue" or color == "b"):
            self.color = "B"
        elif(color == 1 or color == "Y" or color == "yellow" or color == "y"):
            self.color = "Y"                 
        else:
            self.color = "None"
        pass
    
    def get_dist(self, node_):
        # 유클리디안 거리
        #dist = (self.x-node_.x)**2 + (self.y-node_.y)**2
        # 맨하탄 거리
        dist = abs(self.x-node_.x) + abs(self.y-node_.y)
        return dist
    
class NodeList:
    def __init__(self):
        self.length = 0
        self.color = []
        self.nodes = []
        pass

    def add_node(self, node_ : Node):
        self.nodes.append(node_)
        self.length = self.length + 1
        pass

    def get_node_info(self, num : int):
        x = self.nodes[num].x 
        y = self.nodes[num].y
        color = self.nodes[num].color
        return x,y,color
    
    def get_node_color(self, num : int):
        return self.nodes[num].color
    
    def get_node_x(self, num : int):
        return self.nodes[num].x
    
    def get_node_y(self, num : int):
        return self.nodes[num].y

    def get_list_xy(self):
        xs = np.empty((0,1))
        for i in range(self.length):
            xs = np.append(xs, self.nodes[i].x)
            
        ys = np.empty((0,1))
        for i in range(self.length):
            ys = np.append(ys, self.nodes[i].y)
            
        xy_array = np.column_stack((xs, ys))
        return xy_array
    
    def get_list_x(self): # x점들 반환형 : np array
        xs = np.empty((0,1))
        for i in range(self.length):
            xs = np.append(xs, self.nodes[i].x) 
        return xs

    def get_list_y(self):
        ys = np.empty((0,1))
        for node_ in self.nodes:
            ys = np.append(ys, node_.y)
        return ys

    def sort_from_zero(self):
        #원점에서 제일 가까운 점, 그리고 그 점에서 제일 가까운 점 ...
        temp_node = Node(-0.5,0)
        temp_nodes = copy.deepcopy(self.nodes)
        sorted_nodes = []
        
        for i in range(len(self.nodes)):
            closest_node = self.find_closest_node(temp_node, temp_nodes) #temp_nodes 안에 았는 node 중 temp_node와 제일 가까운 점 찾기
            temp_nodes.remove(closest_node)
            sorted_nodes.append(closest_node)
            temp_node = closest_node
        
        self.nodes = sorted_nodes.copy()
        pass
    
    def find_closest_node(self, node_compared, nodes_):
        closest_dist = float('inf')
        for i in range(len(nodes_)):
                cur_dist = node_compared.get_dist(nodes_[i])
                if(cur_dist < closest_dist):
                    closest_dist = cur_dist
                    closest_node = nodes_[i]                
        return closest_node


class DelaunayTriPath:
    def __init__(self, nodelist_ : NodeList):
        self.cones = nodelist_
        self.cones_x = nodelist_.get_list_x()
        self.cones_y = nodelist_.get_list_y()

        # path를 만드는 과정에서 같은 색으로 이루어진 들로네 삼각형을 배제할 것이기때문
        self.deltri = np.empty((0,3))
        self.good_deltri = np.empty((0,3))
        
        # midpoints 저장할 공간
        self.midpoints = NodeList()
        self.midpoints_x = np.empty((0,1))
        self.midpoints_y = np.empty((0,1))

        # path 저장할 공간
        self.path_x = []
        self.path_y = []
        pass
    
    def delaunay_tri_all_cones(self): 
        # 모든 점에 대해서 들로네 삼각분할 시행
        delaunay_triangles = mtri.Triangulation(self.cones_x, self.cones_y)
        self.deltri = delaunay_triangles.get_masked_triangles() # [[a,b,c], [d,e,f], ...]
        # print(f"Delaunay Triangles : \n{self.deltri}\n")
        pass
    
    def delaunay_tri(self):
        self.delaunay_tri_all_cones()
        # 모든 들로네 삼각형 중 같은 색으로 이루어진 bad del tri를 제외하고 고려
        for t in self.deltri:
            a_ind,b_ind,c_ind = t[0],t[1],t[2]
            a_color = self.cones.get_node_color(a_ind)
            b_color = self.cones.get_node_color(b_ind)
            c_color = self.cones.get_node_color(c_ind)

            if(a_color == b_color == c_color):
                continue
            else:
                self.good_deltri = np.append(self.good_deltri, np.array([t]), axis=0)

        pass
    
    def get_delaunay_tri_in_track(self):
        # 1. 원점에서 제일 가까운 들로네 삼각형 찾기 (삼각형의 중심을 이용)
        cloesst_distance_from_zero = float('inf')
        closest_triangle = None
        temp_triangle = np.empty((0,3)) 
        
        for t in self.good_deltri:
            a_ind, b_ind, c_ind = (int)(t[0]),(int)(t[1]),(int)(t[2])
            a_x, a_y = self.cones.get_node_x(a_ind), self.cones.get_node_y(a_ind)
            b_x, b_y = self.cones.get_node_x(b_ind), self.cones.get_node_y(b_ind)
            c_x, c_y = self.cones.get_node_x(c_ind), self.cones.get_node_y(c_ind)

            cx = (a_x + b_x + c_x) / 3
            cy = (a_y + b_y + c_y) / 3
            
            # 원점과 중심점 사이의 거리 계산
            # 유클리디안 거리
            # distance_from_zero = math.sqrt(cx**2 + cy**2)
            # 맨하탄 거리
            distance_from_zero = abs(cx) + abs(cy)
            
            # 현재까지 찾은 삼각형 중 가장 가까운 삼각형보다 더 가까운 삼각형을 찾으면 업데이트
            if(distance_from_zero < cloesst_distance_from_zero):
                cloesst_distance_from_zero = distance_from_zero
                closest_triangle = t
        
        temp_triangle = np.append(temp_triangle, np.array([closest_triangle]), axis=0)
        self.good_deltri = np.delete(self.good_deltri, np.where((self.good_deltri == closest_triangle).all(axis=1))[0][0], axis=0)
        
        # 2. 가장 가까운 들로네 삼각형을 시작으로, 맞닿아 있는 선 중 다른 색깔의 점으로만 이루어진 삼각형을 이어준다.
        # 가장 가까운 들로네 삼각형과 인덱스가 두 개 겹치는 삼각형을 찾고, 그 인덱스가 서로 다른 삼각형이면 next_triangle로 지정
        length_ = len(self.good_deltri)
        for i in range(length_):
            for t in self.good_deltri:
                common_indexs = [value for value in closest_triangle if value in t]
                #두 인덱스가 겹치면
                if len(common_indexs) >=2:
                    com_ind_1, com_ind_2= (int)(common_indexs[0]) , (int)(common_indexs[1])
                    #두 인덱스의 색깔이 다르면 다음 삼각형으로 취급
                    if(self.cones.get_node_color(com_ind_1) != self.cones.get_node_color(com_ind_2)):
                        temp_triangle = np.append(temp_triangle, np.array([t]), axis= 0)
                        self.good_deltri = np.delete(self.good_deltri, np.where((self.good_deltri == t).all(axis=1))[0][0], axis=0)
                        closest_triangle = t
                    elif(self.cones.get_node_color(com_ind_1) == self.cones.get_node_color(com_ind_2)):
                        continue
                    
        self.good_deltri = temp_triangle
        pass
    
    def get_mid(self):
        if(len(self.good_deltri) == 0): self.delaunay_tri()
        self.get_delaunay_tri_in_track()

        # 현실적인 주행에서 보정해주기 위한 점
        self.midpoints.add_node( Node(-0.1,0))
        self.midpoints.add_node( Node(0,0))

        for t in self.good_deltri:
            # print(f"현재 삼각형 : {t}")
            a_ind, b_ind, c_ind = (int)(t[0]),(int)(t[1]),(int)(t[2])
            a_color, a_x, a_y = self.cones.get_node_color(a_ind), self.cones.get_node_x(a_ind), self.cones.get_node_y(a_ind)
            b_color, b_x, b_y = self.cones.get_node_color(b_ind), self.cones.get_node_x(b_ind), self.cones.get_node_y(b_ind)
            c_color, c_x, c_y = self.cones.get_node_color(c_ind), self.cones.get_node_x(c_ind), self.cones.get_node_y(c_ind)
            if(a_color != b_color): 
                self.midpoints.add_node( Node(((a_x+b_x)/2), ((a_y+b_y)/2)) ) 
                # print(f"{a_ind}, {b_ind} : {a_color}, {b_color}")
            if(b_color != c_color): 
                self.midpoints.add_node( Node(((b_x+c_x)/2), ((b_y+c_y)/2)) )
                # print(f"{b_ind}, {c_ind} : {b_color}, {c_color}")
            if(c_color != a_color): 
                self.midpoints.add_node( Node(((c_x+a_x)/2), ((c_y+a_y)/2)) )
                # print(f"{c_ind}, {a_ind} : {c_color}, {a_color}")
        # 원점에서 가장 가까운 점 -> 거기서 제일 가까운 점 -> 또 거기서 제일 가까운 점
        
        self.midpoints.sort_from_zero()
        
        self.midpoints_x = self.midpoints.get_list_x()
        self.midpoints_y = self.midpoints.get_list_y()
        
        return self.midpoints_x, self.midpoints_y
    
    def make_path_bspline(self):
        cv = self.midpoints.get_list_xy()
        p = self.bspline(cv,n=300,degree=5)
        self.path_x, self.path_y = p.T
        pass
        
    def bspline(self, cv, n=100, degree=3):
        import scipy.interpolate as si
        """ Calculate n samples on a bspline

            cv :      Array ov control vertices
            n  :      Number of samples to return
            degree:   Curve degree
        """
        cv = np.asarray(cv)
        count = cv.shape[0]

        # Prevent degree from exceeding count-1, otherwise splev will crash
        degree = np.clip(degree,1,count-1)

        # Calculate knot vector
        kv = np.array([0]*degree + list(range(count-degree+1)) + [count-degree]*degree,dtype='int')

        # Calculate query range
        u = np.linspace(0,(count-degree),n)

        # Calculate result
        return np.array(si.splev(u, (kv,cv.T,degree))).T
    
    def get_path_x(self):
        return self.path_x
    
    def get_path_y(self):
        return self.path_y
    
    
    
def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def callback(msg):
    points = []
    
    cone_list = NodeList()
    
    # Get points from subscriber node
    for i in range(len(msg.poses)):
        points.append([])
        points[i].append(msg.poses[i].pose.position.x)
        points[i].append(msg.poses[i].pose.position.y)
        points[i].append(msg.poses[i].pose.position.z) # 색깔 
    
    for point in points:
        cone_list.add_node(Node(point[0],point[1],point[2]))
        
    # Find the midpoints and the path using delaunay triangulation
    deltri = DelaunayTriPath(cone_list)
    deltri.delaunay_tri()
    midpoints_x , midpoints_y = deltri.get_mid()

    deltri.make_path_bspline()
    path_x = deltri.get_path_x()
    path_y = deltri.get_path_y()
    
    # Publish Midpoints
    midpoint_msg = Marker()
    midpoint_msg.header.frame_id = 'odom'
    midpoint_msg.type = Marker.POINTS
    midpoint_msg.action = Marker.ADD
    
    for i in range(len(midpoints_x)):
        midpoint_msg.points.append(Point(midpoints_x[i], midpoints_y[i], 0))
        
    midpoint_msg.color = ColorRGBA(1, 1, 1, 1)
    midpoint_msg.scale.x = 0.1
    midpoint_msg.scale.y = 0.1
    midpoint_msg.scale.z = 0
    
    mid_pub.publish(midpoint_msg)
    
    # Publish Path
    path_msg = Path()
    path_msg.header.frame_id = 'odom'
    
    for i in range(len(path_x)-1):
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = path_x[i]
        pose_stamped.pose.position.y = path_y[i]
        pose_stamped.pose.position.z = 0
        
        yaw = np.arctan2(path_y[i+1]-path_y[i],path_x[i+1]-path_x[i])
        qx_,qy_,qz_,qw_ = get_quaternion_from_euler(0,0,yaw)
        
        pose_stamped.pose.orientation.x = qx_
        pose_stamped.pose.orientation.y = qy_
        pose_stamped.pose.orientation.z = qz_
        pose_stamped.pose.orientation.w = qw_
        path_msg.poses.append(pose_stamped)
    
    path_pub.publish(path_msg)
    
     # Publish cones
    conepoint_msg = Marker()
    conepoint_msg.header.frame_id = 'odom'
    conepoint_msg.type = Marker.POINTS
    conepoint_msg.action = Marker.ADD
    
    for point in points:
        conepoint_msg.points.append(Point(point[0], point[1], 0))
        if(point[2] == 0):
            conepoint_msg.colors.append(ColorRGBA(0,0,1,1))
        else:
            conepoint_msg.colors.append(ColorRGBA(1,1,0,1))
            
    conepoint_msg.scale.x = 0.1
    conepoint_msg.scale.y = 0.1
    conepoint_msg.scale.z = 0

    cone_pub.publish(conepoint_msg)
    
if __name__ =="__main__":
    rospy.init_node('path_generator')
    
    rospy.Subscriber("first_lap_path",Path,callback)

    path_pub = rospy.Publisher("waypoints",Path,queue_size=10)
    mid_pub = rospy.Publisher("mid_points",Marker,queue_size=10)
    cone_pub = rospy.Publisher("cone_point",Marker,queue_size=10)
    self.yellow_path_pub = self.create_publisher(Path, '/fastlap_path', 10)
    self.blue_path_pub = self.create_publisher(Path, '/fastlap_path', 10)
    rospy.spin()