#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np

import matplotlib.tri as mtri
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,PoseStamped
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path

class node:
    count = 0
    def __init__(self, x : float, y : float, color : str):
        self.x = x                              
        self.y = y                              
        self.color = color                      
        self.index = node.count                 

        if(node.count == 0): 
            node.nodelist = nodelist()

        node.count = node.count + 1             
        node.nodelist.AddNode(self) 
        pass

class nodelist:
    def __init__(self) -> None:
        nodelist.count = 1
        self.x = np.empty((0,1))
        self.y = np.empty((0,1))
        self.color = list()
        self.nodes = []
        pass

    def AddNode(self, node_ : node):
        self.nodes.append(node_)
        self.x = np.append(self.x, node_.x)
        self.y = np.append(self.y, node_.y)
        nodelist.count = nodelist.count + 1
        pass

    def GetInfo(self, num : int):
        x = self.nodes[num].x
        y = self.nodes[num].y
        color = self.nodes[num].color
        return x,y,color
    
    def GetInfoColor(self, num : int):
        return self.nodes[num].color
    
    def GetInfoX(self, num : int):
        return self.nodes[num].x
    
    def GetInfoY(self, num : int):
        return self.nodes[num].y

    def GetInfoLength(self):
        length_ : int
        length_ = len(self.nodes)
        return length_
    
class DelaunayTriPath:
    def __init__(self, nodelist_ : nodelist) -> None:
        self.nodelist_ = nodelist_
        self.x = np.array(nodelist_.x)
        self.y = np.array(nodelist_.y)
        self.bad_deltri = np.empty((0,3))
        self.good_deltri = np.empty((0,3)) #midpoin
        self.midpoint = np.empty((0,2))
        print(f"ë“¤ë¡œë„¤ ì‚¼ê°ë¶„í• ì„ í†µí•œ ê²½ë¡œ ê³„íš ì‹œí–‰")
        print(f"0ë²ˆ ë…¸ë“œ : {nodelist_.GetInfo(0)}")
        print(f"1ë²ˆ ë…¸ë“œ : {nodelist_.GetInfo(1)}\n")

    def DelaunayTriFirst(self): #
        delaunay_triangles = mtri.Triangulation(self.x, self.y)
        self.deltri = delaunay_triangles.get_masked_triangles() # [[a,b,c], [d,e,f], ...]
        print(f"ë¸ë¡œë„¤ì‚¼ê°í˜• ì¸ë±ìŠ¤ : \n{self.deltri}\n")

    def DelaunayTri(self): #  DelaunayTriangulation
        self.DelaunayTriFirst()
        for t in self.deltri:
            a_ind,b_ind,c_ind = t[0],t[1],t[2]
            a_color = self.nodelist_.GetInfoColor(a_ind)
            b_color = self.nodelist_.GetInfoColor(b_ind)
            c_color = self.nodelist_.GetInfoColor(c_ind)
            print(f"{t} : {a_color, b_color, c_color}\n")

            if(a_color == b_color == c_color):
                self.bad_deltri = np.append(self.bad_deltri, np.array([t]), axis=0)
            else:
                self.good_deltri = np.append(self.good_deltri, np.array([t]), axis=0)

        print(f"good del tri : \n{self.good_deltri}\n")
        pass
        
    def GetMid(self): # good_de
        if(len(self.good_deltri) == 0): self.DelaunayTri() # DelaunayTr

        for t in self.good_deltri:
            a_ind, b_ind, c_ind = (int)(t[0]),(int)(t[1]),(int)(t[2])
            a_color, a_x, a_y = self.nodelist_.GetInfoColor(a_ind), self.nodelist_.GetInfoX(a_ind), self.nodelist_.GetInfoY(a_ind)
            b_color, b_x, b_y = self.nodelist_.GetInfoColor(b_ind), self.nodelist_.GetInfoX(b_ind), self.nodelist_.GetInfoY(b_ind)
            c_color, c_x, c_y = self.nodelist_.GetInfoColor(c_ind), self.nodelist_.GetInfoX(c_ind), self.nodelist_.GetInfoY(c_ind)

            if(a_color == b_color): continue
            else: self.midpoint = np.append(self.midpoint, np.array([[(a_x+b_x)/2, (a_y+b_y)/2]]), axis=0)
            if(b_color == c_color): continue
            else: self.midpoint = np.append(self.midpoint, np.array([[(b_x+c_x)/2, (b_y+c_y)/2]]), axis=0)
            if(c_color == a_color): continue
            else: self.midpoint = self.midpoint.append([(c_x+a_x)/2, (c_y+a_y)/2], axis=0)

        print(f"midpoint : \n{self.midpoint}\n")
        return self.midpoint

    def GetMidX(self): #midpoin
        if(len(self.midpoint)==0): self.GetMid() # GetMid(
        print(f"midpoint x : {self.midpoint[:,0]}\n")
        return self.midpoint[:,0]
    
    def GetMidY(self): #midpoin
        if(len(self.midpoint)==0): self.GetMid()
        print(f"midpoint y : {self.midpoint[:,1]}\n")
        return self.midpoint[:,1]
    
#points = [[0, 0, 0], [1, 0, 0], [2, 0.5, 0],[3, 1, 0], [4, 1.5, 0 ],[5,2.5, 0], [0,1,1],[1,1,1],[2,1.5,1],[3,2,1],[4,2.5,1],[5,8,1]]

def callback(msg):
    points=[]

    for i in range(len(msg.poses)):
        points.append([])
        points[i].append(msg.poses[i].pose.position.x)
        points[i].append(msg.poses[i].pose.position.y)
        points[i].append(msg.poses[i].pose.position.z)
    
    print(points)
    
    # Get points from designated matrix
    for point in points:
        x_ = point[0]
        y_ = point[1]
        color_code = point[2]
        if(color_code == 0):
            node(x_,y_,"B")
        else:
            node(x_,y_,"Y")
    
    # Find the midpoints from the path
    deltri_path = DelaunayTriPath(node.nodelist)
    deltri_path.DelaunayTri()
    midpoint_x = deltri_path.GetMidX()
    midpoint_y = deltri_path.GetMidY()
    
    # Midpoint
    midpoint_msg = Marker()
    midpoint_msg.header.frame_id = 'odom'
    midpoint_msg.type = Marker.POINTS
    midpoint_msg.action = Marker.ADD
        
    for i in range(len(midpoint_x)):
        midpoint_msg.points.append(Point(midpoint_x[i], midpoint_y[i], 0))
    
    midpoint_msg.color = ColorRGBA(1, 1, 1, 1)
    midpoint_msg.scale.x = 0.1
    midpoint_msg.scale.y = 0.1
    midpoint_msg.scale.z = 0
    
    mid_pub.publish(midpoint_msg)

if __name__=='__main__':
    rospy.init_node('path_generator')
    
    rospy.Subscriber("my_path",Path,callback)

    #path_pub = rospy.Publisher("/waypoints",Path,queue_size=10)
    mid_pub = rospy.Publisher("cone_mid_points",Marker,queue_size=10)

    rospy.spin()