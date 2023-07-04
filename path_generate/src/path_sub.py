#!/usr/bin/env python

import rospy
import struct
import math
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseArray, PoseStamped 
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Path, Odometry
import numpy as np

minimum_points = [] #

def clustering_callback2(msg):
    global cone_centers
    cone_centers = PoseArray()
    for i in range(len(msg.poses)) :
        if ( abs(msg.poses[i].position.x) < 0.8 and abs(msg.poses[i].position.y) < 0.8 ):
            continue
        
        elif ( (msg.poses[i].position.x>-0.5) and (msg.poses[i].position.x < 6.5 ) and abs(msg.poses[i].position.y) < 2.0 ) :
            cone_centers.poses.append(msg.poses[i])
            continue
        
            #pose = PoseStamped()
            #if abs(msg.poses[i].position.y) < 2.85 and msg.poses[i].position.x < 1 and msg.poses[i].orientation.z > 0.3 :
            #    pose.pose.position.x = msg.poses[i].position.x
            #    pose.pose.position.y = msg.poses[i].position.y
            #    pose.pose.position.z = 1 if pose.pose.position.y > 0 else 0
            #    path.poses.append(pose)
            

def id_points_callback(msg):

    c = msg.header.frame_id[0]
        
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
            PointField('intensity', 16, PointField.FLOAT32, 1)]
    
    cone_number = msg.header.frame_id[1:3]
    
    if cone_number == '0':
        if not len(minimum_points) == 0 :
            sorted_point_pub.publish(pc2.create_cloud(sorted_points.header,fields,minimum_points))
            
            path = Path()
            path.header = msg.header   
            path.header.stamp = rospy.Time.now() 
            path.header.frame_id = 'velodyne'
            #print(minimum_points)
            #print(minimum_points)
            #minimum_points = [[-6, 10, 1], [6, -8, 1], [-2, 7, -1], [-3, -3, -1], [10, -2, 0], [2, -1, 0]]
            minimum_points = sorted(minimum_points, key=lambda x: abs(x[1]), reverse=True)
            print(minimum_points)
            for i in range(len(minimum_points)):
                pose = PoseStamped()
                pose.pose.position.x = minimum_points[i][0]
                pose.pose.position.y = minimum_points[i][1]
                pose.pose.position.z = minimum_points[i][4]
                path.poses.append(pose)
                
            path_pub.publish(path)
            print("publish")
            del minimum_points[:]


    cone_x = []
    cone_y = []
    cone_z = []

    cone_dist = []
    for p in pc2.read_points(msg , field_names = ("x","y","z"), skip_nans = True):
        cone_x.append(p[0])
        cone_y.append(p[1])
        cone_z.append(p[2])

    if (len(cone_centers.poses)) > 0 : 
        for i in range(len(cone_centers.poses)) :  # 일단 라이더 roi에 있는 것은 -1.0 으로 두고 센서퓨전 된것에만 새롭게 입히는 걸로 
            
            pt[4] = -1.0
            r = int(1 * 255.0)
            g = int(1 * 255.0)
            b = int(0 * 255.0)
            a = 255
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            pt[3] = rgb
            
            for j in range(len(cone_x)) : 
                try:
                    if math.sqrt((cone_x[j]-cone_centers.poses[i].position.x)**2 + (cone_y[j]-cone_centers.poses[i].position.y)**2) < 0.2 : # ??? 여기서 생기는  IndexError: list index out of range 에러는 뭔지 모르곘음
                        
                        pt = [round(cone_centers.poses[i].position.x, 2), round(cone_centers.poses[i].position.y, 2),0,0,0]
                        
                        if c == "b" : 
                            pt[4] = 0.0
                        elif c == "y" :
                            pt[4] = 1.0
                            
                        r = int(1 * 255.0)
                        g = int(1 * 255.0)
                        b = int(0 * 255.0)
                        a = 255
                        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                        pt[3] = rgb
                        #minimum_points.append(pt)
                        break

                except IndexError:
                    pass
                
            minimum_points.append(pt)
                        
    sorted_points.header = msg.header   
    sorted_points.header.stamp = rospy.Time.now()   
    sorted_points.header.frame_id = 'velodyne'
        
    
if __name__=='__main__':

    rospy.init_node('cone_distance')
    rospy.Subscriber("/id_points2",PointCloud2,id_points_callback)
    rospy.Subscriber("/adaptive_clustering/poses",PoseArray,clustering_callback2)
    
    sorted_point_pub = rospy.Publisher("/sorted_points2",PointCloud2,queue_size=1)
    path_pub = rospy.Publisher("/sensor_fusion_output",Path,queue_size=1) # 라바콘의 위치와 색깔 
    
    cone_centers = PoseArray()
    sorted_points = PointCloud2()
    path = Path()
    
    rospy.spin()

