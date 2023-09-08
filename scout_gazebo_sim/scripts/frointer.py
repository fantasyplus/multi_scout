#!/usr/bin/env python3
# coding=utf-8 

import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from copy import copy
import mean_shift as ms
import time
import matplotlib.pyplot as plt
def Frointer(Map):
    M,N = Map.shape
    result = []
    for i in range(1,M-1):
        for j in range(1,N-1):
            if Map[i,j] <0.5:
                # if (Map[i-1,j] ==0.5) or (Map[i+1,j] ==0.5) or (Map[i,j+1] ==0.5) \
                #     or (Map[i,j-1] ==0.5) or (Map[i-1,j+1] ==0.5) or (Map[i-1,j-1] ==0.5) \
                #     or (Map[i+1,j-1] ==0.5) or (Map[i+1,j+1] ==0.5):
                if (Map[i-1,j] ==0.5) or (Map[i+1,j] ==0.5) or (Map[i,j+1] ==0.5) or (Map[i,j-1] ==0.5):
                    result.append([i,j])                    
    return result

def Cluster(Frontiers):
    Clusters = []
    while len(Frontiers) != 0:
        open_list = []
        close_list = []
        open_list.append(Frontiers[0])
        close_list.append(Frontiers[0])
        Frontiers.pop(0)
        while len(open_list) != 0:
            for i in range(-1,2):
                for j in range(-1,2):
                    if [open_list[0][0]+i,open_list[0][1]+j] in Frontiers:
                        Frontiers.remove([open_list[0][0]+i,open_list[0][1]+j])
                        open_list.append([open_list[0][0]+i,open_list[0][1]+j])
                        close_list.append([open_list[0][0]+i,open_list[0][1]+j])
            open_list.pop(0)        
        Clusters.append(close_list)
    return Clusters
    
def mapCallback(msg):
    data = np.array(msg.data)
    width = msg.info.width
    height = msg.info.height
    map = []
    for i in range(height):
        map.append(data[i*width:(i+1)*width])
    map = np.array(map)
    resolution, origin_x, origin_y = msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y
    map1 = (map==100).astype(np.int)
    map2 = (map==-1).astype(np.int)*0.5
    map = np.maximum(map1,map2)
    m = np.ones((map.shape[0]+10,map.shape[1]+10))*0.5
    m[5:map.shape[0]+5,5:map.shape[1]+5] = map


    temp = Frointer(m)
    clust = Cluster(temp)
    clust_len = len(clust)


    Clust = []
    mean_shifter = ms.MeanShift()
    for i in range(clust_len):
        if len(clust[i])<15:
            continue
        mean_shift_result = mean_shifter.cluster(np.array(clust[i])/20, kernel_bandwidth = 1)
        cluster_assignments = mean_shift_result.cluster_ids
        for j in range(np.unique(cluster_assignments).size):
            ind = np.argwhere(cluster_assignments == j)
            xx,yy=0.0,0.0
            for k in ind:
                xx +=clust[i][k[0]][0]
                yy +=clust[i][k[0]][1]
            xx /=len(ind)
            yy /=len(ind)
            Clust.append([int(xx),int(yy)])
            

    points = Marker()       
    points.header.frame_id = msg.header.frame_id
    points.header.stamp = rospy.Time(0)
    points.ns = "markers2"
    points.id = 0
    points.type = Marker.POINTS
    points.action = Marker.ADD
    points.pose.orientation.w = 1.0
    points.scale.x = 0.2
    points.scale.y = 0.2
    # points.scale.x = 2
    # points.scale.y = 2
    # points.scale.z = 0.2
    # points.scale.x = 0.05
    # points.scale.y = 0.05
    points.color.r = 255.0/255.0
    points.color.g = 0.0/255.0
    points.color.b = 255.0/255.0
    points.color.a = 1
    points.lifetime = rospy.Duration()
    p = Point()
    p.z = 0
    pp = []

    # Clust = clust #             初始边界点
    # for i in range(clust_len):
    #     if len(Clust[i])<10:
    #         continue
    #     for j in range(len(Clust[i])):
    #         p.x = (Clust[i][j][1]-5)*resolution+(origin_x)
    #         p.y = (Clust[i][j][0]-5)*resolution+(origin_y)
    #         pp.append(copy(p))   

    for j in range(len(Clust)):
        p.x = (Clust[j][1]-5)*resolution+(origin_x)
        p.y = (Clust[j][0]-5)*resolution+(origin_y)
        pp.append(copy(p))   

    # rospy.loginfo(len(pp))
    # rospy.loginfo(pp)
    points.points = pp
    pub.publish(points)

if __name__ == "__main__":
    try:
        rospy.init_node('frointer', anonymous=True)
        pub = rospy.Publisher('/frointer', Marker, queue_size=1) 
        rospy.Subscriber("/map", OccupancyGrid, mapCallback, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass