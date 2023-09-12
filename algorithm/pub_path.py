#!/usr/bin/env python

import json
from os import path
import threading
import time

import rospy
import actionlib
import tf 

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# 创建全局锁
state_lock = threading.Lock()
client_lock = threading.Lock()

# 用于保存到达时间和间隔的全局字典
arrival_times = {} 

# 创建全局 Barrier 对象
barrier = None

def readPaths():
    json_path=path.dirname(__file__)+"/data.json"
    with open(json_path,'r') as f:
        paths=json.load(f)
    return paths

def setModelState(model_name,x,y):
    mode_pub=rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=1)
    try:
        model_state=ModelState()
        model_state.model_name=model_name
        model_state.pose.position.x=x
        model_state.pose.position.y=y
        model_state.pose.position.z=0.26
        q=quaternion_from_euler(0,0,1.5707)
        model_state.pose.orientation=q

        mode_pub.publish(model_state)
    except rospy.ROSException as e:
        print("in setModelState",e)
        pass

def getModelState(model_name,tf_listener):
    with state_lock:
        try:
            tf_listener.waitForTransform(model_name + '/base_link', model_name+'/odom', rospy.Time(), rospy.Duration(4.0))
            (trans,rot)=tf_listener.lookupTransform(model_name+'/base_link',model_name+'/odom',rospy.Time(0))

            x=trans[0]
            y=trans[1]
            yaw=euler_from_quaternion(rot)[2]*-1
            # print(f"get {model_name} state",x,y,yaw)
            return x,y,yaw
        except rospy.ServiceException as e:
            print("in getModelState",e)
            pass
        pass


def moveBaseClient(robot_name,goal_x,goal_y,yaw):
    with client_lock:
        move_base_name=robot_name+'/move_base'
        client=actionlib.SimpleActionClient(move_base_name,MoveBaseAction)
        client.wait_for_server()

        goal=MoveBaseGoal()
        frame_id=robot_name+'/odom'
        goal.target_pose.header.frame_id=frame_id
        goal.target_pose.header.stamp=rospy.Time.now()
        goal.target_pose.pose.position.x=goal_x
        goal.target_pose.pose.position.y=goal_y
        q=quaternion_from_euler(0,0,yaw)
        goal.target_pose.pose.orientation.x=q[0]
        goal.target_pose.pose.orientation.y=q[1]
        goal.target_pose.pose.orientation.z=q[2]
        goal.target_pose.pose.orientation.w=q[3]

        client.send_goal(goal)
    pass

    wait=client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_goal_status_text()
        

def processPathsSingleThread(paths):
    for robot_name, path in paths.items():
        processPathForRobot(robot_name, path)

def processPathsMultiThread(paths,tf_listener):
    threads = []
    for robot_name, path in paths.items():
        t = threading.Thread(target=processPathForRobot, args=(robot_name, path,tf_listener))
        threads.append(t)
        t.start()

    for t in threads:
        t.join()  


def processPathForRobot(robot_name, path, tf_listener):
    global arrival_times
    if robot_name not in arrival_times:
        arrival_times[robot_name] = {'intervals': [], 'arrival_times': []}

    robot_state = getModelState(robot_name, tf_listener)
    if robot_state == None:
        print("get {} state failed".format(robot_name))
        return

    robot_yaw = robot_state[2]
    start_time = None  # 初始化开始时间

    for point in path:
        if start_time is None:
            start_time = time.time()  # 记录第一个目标点的发送时间
            arrival_times[robot_name]['arrival_times'].append(0)  # 第一个点的到达时间为0

        print("----------------------------------")
        print("send goal to ", robot_name, point)

        status = moveBaseClient(robot_name, point[0], point[1], robot_yaw)

        if 'Goal' in status:
            current_time = time.time()  # 获取当前时间
            elapsed_time = current_time - start_time  # 计算从开始到现在经过的时间

            arrival_times[robot_name]['arrival_times'].append(elapsed_time)  # 保存到达当前目标点的时间
            arrival_times[robot_name]['intervals'].append(elapsed_time - arrival_times[robot_name]['arrival_times'][-2])  # 保存到达当前目标点和上一个目标点的时间间隔

            print(f"Robot {robot_name} reached goal at {elapsed_time}s")

            print("Robot {} arrived ,waiting....".format(robot_name))
            barrier.wait()  #等待其他线程

            print("----------------------------------")
            print("send {} next goal".format(robot_name))
        else:
            print("-----error!!!!-----")
            print(status)
            print("exit")
            return

    print("Robot {} finished path".format(robot_name))

if __name__ =='__main__':
    paths=readPaths()

    #init barrier
    num_threads = len(paths)
    barrier = threading.Barrier(num_threads)

    rospy.init_node('pub_path')
    rate=rospy.Rate(100)

    tf_listener=tf.TransformListener()
    while not rospy.is_shutdown():
        processPathsMultiThread(paths,tf_listener)
        # processPathsSingleThread(paths)
        rate.sleep()