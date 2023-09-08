#!/usr/bin/env python
#-*-coding:utf-8-*-

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros
from tf import TransformBroadcaster

class OdometryNode:
    def __init__(self):
        # init internals 初始化内部
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None

        # Set the update rate 设置更新速率
        rospy.Timer(rospy.Duration(.02), self.timer_callback) # 100hz
        #初始化发布方
        self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers 设置订阅话题
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar 找到赛车的索引
        try:
            #创建msg消息对象
            arrayIndex = msg.name.index('scout3::base_link')
            # print(arrayIndex)
        except ValueError as e:
            # Wait for Gazebo to startup 等待Gazebo启动
            pass
        else:
            # Extract our current position information 提取我们当前的位置信息
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
        self.last_recieved_stamp = rospy.Time.now()
        # print('/gazebo/link_states')

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return
        br = TransformBroadcaster()
        br.sendTransform((self.last_received_pose.position.x, self.last_received_pose.position.y, self.last_received_pose.position.z),
                        (self.last_received_pose.orientation.x, self.last_received_pose.orientation.y, 
                        self.last_received_pose.orientation.z, self.last_received_pose.orientation.w),
                        rospy.Time.now(),
                        "scout3/base_link",
                        "odom")    

# Start the node
if __name__ == '__main__':
    rospy.init_node("odom_scout_3")#建立节点
    #调用类。在类调用类的同时调用了发送tf和topic的函数
    node = OdometryNode()
    #等待阻塞
    rospy.spin()

