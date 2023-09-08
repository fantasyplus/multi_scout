#!/usr/bin/env python3
# roslaunch clean_robot nav_slam.launch use_pose_extrapolato:=false 
# rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

from cmath import inf
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time
import numpy as np

position_x, position_y = 0,0
goal = MoveBaseGoal()

def movebase_client(x,y):
	client = actionlib.SimpleActionClient('/move_base',MoveBaseAction)
	client.wait_for_server(rospy.Duration(1))
	goal.target_pose.header.frame_id = "odom"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.orientation.w = 1.0
	client.send_goal(goal)

	for j in range(20):
		wait = client.wait_for_result(rospy.Duration(0.1))
		global position_x, position_y
		if np.hypot(position_x - goal.target_pose.pose.position.x, position_y - goal.target_pose.pose.position.y)<=0.30:
			break

	if not wait:
		client.cancel_goal()
	else:
	    return client.get_result()

def assignGoal(data):
	minDis = inf
	global position_x, position_y
	goalind = 0
	if len(data.points) != 0:
		for i in range(len(data.points)):
			# rospy.loginfo(data.points)
			# if minDis>np.hypot(position_x-data.points[i].x,position_y-data.points[i].y):
			# 	minDis = np.hypot(position_x-data.points[i].x,position_y-data.points[i].y)
			if minDis>abs(position_x-data.points[i].x)+abs(position_y-data.points[i].y):
				minDis = abs(position_x-data.points[i].x)+abs(position_y-data.points[i].y)
				goalind = i
		print(position_x, position_y)
		print(data.points[goalind].x,data.points[goalind].y)
		movebase_client(data.points[goalind].x,data.points[goalind].y)


def odomCallback(msg):
	global position_x, position_y
	position_x, position_y = msg.pose.pose.position.x, msg.pose.pose.position.y

def node():
	rospy.init_node('assignGoal', anonymous=False)
	rospy.Subscriber("/odom", Odometry, odomCallback)
	rospy.sleep(0.5)
	rospy.Subscriber('/frointer', Marker, assignGoal, queue_size=1)
	rospy.spin()


if __name__ == '__main__':
	node()
"""
rostopic pub /tb3_0/move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
roslaunch multi_turtlebot3_navigation move_base_namespace.launch robot_namespace:="tb3_0"

"""