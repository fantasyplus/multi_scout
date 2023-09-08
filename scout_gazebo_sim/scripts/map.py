#!/usr/bin/env python3
# coding=utf-8 

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import matplotlib.pyplot as plt
  
def mapCallback(msg):
    data = np.array(msg.data)
    width = msg.info.width
    height = msg.info.height
    map = []
    for i in range(height):
        map.append(data[i*width:(i+1)*width])
    map = np.array(map)

    # resolution, origin_x, origin_y = msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y
    map1 = (map==100).astype(np.int)
    map2 = (map==-1).astype(np.int)*0.5
    map = np.maximum(map1,map2)
    m = map

    # m1 = np.sign(np.add(np.sign(np.subtract(m, 0.6)),1))
    # kernel = np.ones((5,5), np.uint8)
    # # kernel1 = np.ones((3,3), np.uint8)
    # # m11 = cv2.dilate(m1, kernel1)
    # m1 = cv2.dilate(m1, kernel)
    m2 = np.sign(np.add(np.sign(np.subtract(m, 0.4)),1))
    kernel = np.ones((5,5), np.uint8)
    m2 = cv2.dilate(m2, kernel)
    m2 = cv2.erode(m2, kernel, iterations=1)
    m = np.maximum(m2 * 0.5,m)

    # plt.imshow(m,cmap = 'gray')
    # plt.savefig('DR1.png')

    map_msg = OccupancyGrid()
    map_msg = msg
    map_msg.header.stamp = rospy.Time.now()
    map_msg.info.map_load_time = rospy.Time.now()
    msg.header.frame_id = "map"
    m = (m>=0.7).astype(np.int)*100 + (m==0.5).astype(np.int)*-1
    map_msg.data = [i for row in m.tolist() for i in row]



    pub.publish(map_msg)

if __name__ == "__main__":
    try:
        rospy.init_node('frointer', anonymous=True)
        pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1) 
        rospy.Subscriber("/scout1/projected_map", OccupancyGrid, mapCallback, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass