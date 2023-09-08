#!/usr/bin/env python3
# Move turtlebot3 from location A to location B through move_base, action client
# roslaunch clean_robot nav_slam.launch use_pose_extrapolato:=false 
# rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
# rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

from cmath import inf
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time
import numpy as np
from scipy.optimize import linear_sum_assignment
from gazebo_msgs.msg import LinkStates

position_x, position_y = [0,0,0],[0,0,0]
goal = MoveBaseGoal()

# def movebase_client(point0, point1, point2):
# 	client0 = actionlib.SimpleActionClient('/scout1/move_base',MoveBaseAction)
# 	client1 = actionlib.SimpleActionClient('/scout2/move_base',MoveBaseAction)
# 	client2 = actionlib.SimpleActionClient('/scout3/move_base',MoveBaseAction)
# 	client0.wait_for_server()
# 	client1.wait_for_server()
# 	client2.wait_for_server()
# 	goal.target_pose.header.frame_id = "odom"
# 	goal.target_pose.header.stamp = rospy.Time.now()
# 	goal.target_pose.pose.position.x = point0.x
# 	goal.target_pose.pose.position.y = point0.y
# 	goal.target_pose.pose.orientation.w = 1.0
# 	client0.send_goal(goal)
# 	goal.target_pose.header.frame_id = "odom"
# 	goal.target_pose.header.stamp = rospy.Time.now()
# 	goal.target_pose.pose.position.x = point1.x
# 	goal.target_pose.pose.position.y = point1.y
# 	goal.target_pose.pose.orientation.w = 1.0
# 	client1.send_goal(goal)
# 	goal.target_pose.header.frame_id = "odom"
# 	goal.target_pose.header.stamp = rospy.Time.now()
# 	goal.target_pose.pose.position.x = point2.x
# 	goal.target_pose.pose.position.y = point2.y
# 	goal.target_pose.pose.orientation.w = 1.0
# 	client2.send_goal(goal)
# 	wait = client0.wait_for_result(rospy.Duration(1))
def movebase_client(point0, point1):
	client0 = actionlib.SimpleActionClient('/scout1/move_base',MoveBaseAction)
	client1 = actionlib.SimpleActionClient('/scout2/move_base',MoveBaseAction)
	client0.wait_for_server()
	client1.wait_for_server()

	goal.target_pose.header.frame_id = "odom"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = point0.x
	goal.target_pose.pose.position.y = point0.y
	goal.target_pose.pose.orientation.w = 1.0
	client0.send_goal(goal)

	goal.target_pose.header.frame_id = "odom"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = point1.x
	goal.target_pose.pose.position.y = point1.y
	goal.target_pose.pose.orientation.w = 1.0
	client1.send_goal(goal)

	wait = client0.wait_for_result(rospy.Duration(1))
	client0.cancel_goal()
	client1.cancel_goal()

def assignGoal1(data):
	client0 = actionlib.SimpleActionClient('/scout1/move_base',MoveBaseAction)
	client1 = actionlib.SimpleActionClient('/scout2/move_base',MoveBaseAction)
	client0.wait_for_server()
	client1.wait_for_server()

	goal.target_pose.header.frame_id = "odom"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 2
	goal.target_pose.pose.position.y = 28
	goal.target_pose.pose.orientation.w = 1.0
	client0.send_goal(goal)

	goal.target_pose.header.frame_id = "odom"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 26
	goal.target_pose.pose.position.y = 16
	goal.target_pose.pose.orientation.w = 1.0
	client1.send_goal(goal)

	wait = client0.wait_for_result(rospy.Duration(1))
	client0.cancel_goal()
	client1.cancel_goal()


def assignGoal(data):
	# if(len(data.points)>0):
	# 	n = 2	
	# 	frontiers = data.points
	# 	global position_x, position_y
	# 	cost = np.ones((n,len(frontiers)))*np.inf
	# 	for j in range(len(frontiers)):
	# 		for i in range(n):
	# 			delx = int(np.abs(position_x[i]-frontiers[j].x))
	# 			dely = int(np.abs(position_y[i]-frontiers[j].y))
	# 			cost[i,j] = 10*delx+10*dely
	# 			# cost[i,j] = 10*np.linalg.norm((delx,dely))

	# 	if len(frontiers)<n:
	# 		cost_assignment = np.zeros((n,n))
	# 		cost_assignment[:,0:len(frontiers)] = cost
	# 	if len(frontiers)==n:
	# 		cost_assignment = np.zeros((n,n))
	# 		cost_assignment[:,:] = cost
	# 	if len(frontiers)>n:
	# 		cost_assignment = np.zeros((len(frontiers),len(frontiers)))
	# 		cost_assignment[0:n,:] = cost

	# 	matches = linear_sum_assignment(cost_assignment)
	# 	# print(cost_assignment)
	# 	for k in range(n):
	# 		if(matches[1][k]>=len(frontiers)):
	# 			matches[1][k] = np.argmin(cost[k])
	# 	print(matches[1][:n])
		movebase_client(data.points[0], data.points[1])
		# movebase_client(frontiers[matches[1][0]], frontiers[matches[1][1]], frontiers[matches[1][2]])

#ding yue dang qian wei zhi
def odomCallback(msg):
	global position_x, position_y
	try:
		#创建msg消息对象
		arrayIndex1 = msg.name.index('scout1::base_link')
		arrayIndex2 = msg.name.index('scout2::base_link')
		# arrayIndex3 = msg.name.index('scout3::base_link')
		# print(arrayIndex)
	except ValueError as e:
		# Wait for Gazebo to startup 等待Gazebo启动
		pass
	else:
		# Extract our current position information 提取我们当前的位置信息
		position_x[0], position_y[0] = msg.pose[arrayIndex1].position.x, msg.pose[arrayIndex1].position.y
		position_x[1], position_y[1] = msg.pose[arrayIndex2].position.x, msg.pose[arrayIndex2].position.y
		# position_x[2], position_y[2] = msg.pose[arrayIndex3].position.x, msg.pose[arrayIndex3].position.y

def node():
	rospy.init_node('assignGoal', anonymous=False)
	# rate = rospy.Rate(1)
	# rospy.Subscriber("/gazebo/link_states", LinkStates, odomCallback)
	rospy.Subscriber("/gazebo/link_states", LinkStates, assignGoal1)
	rospy.Subscriber('/frointer', Marker, assignGoal, queue_size=1) #target fa song zhi /frointer
	# rate.sleep()
	rospy.spin()


if __name__ == '__main__':

    # try:
    #     rospy.init_node('movebase_client_py')
    #     result = movebase_client()
    #     if result:
    #         rospy.loginfo("Reached goal!")
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Interrupt, navigation finnished.")
	# rospy.sleep(30)
	node()
"""
rostopic pub /tb3_0/move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
roslaunch multi_turtlebot3_navigation move_base_namespace.launch robot_namespace:="tb3_0"

"""