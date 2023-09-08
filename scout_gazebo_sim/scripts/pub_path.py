#!/usr/bin/python3

import json
from os import path

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler

def readPaths():
    json_path=path.dirname(__file__)+"/data.json"
    with open(json_path,'r') as f:
        paths=json.load(f)
    return paths

def setModelState(model_name,x,y):
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

def processPaths(paths):
    for name,path in paths.items():
        for point in path:
            print("send goal to ",name,point)
            status=moveBaseClient(name,point[0],point[1])
            print(status)
            while ('GOAL' in status):
                print("send next goal")


def moveBaseClient(robot_name,goal_x,goal_y):
    move_base_name=robot_name+'/move_base'
    client=actionlib.SimpleActionClient(move_base_name,MoveBaseAction)
    client.wait_for_server()

    goal=MoveBaseGoal()
    frame_id=robot_name+'/odom'
    goal.target_pose.header.frame_id=frame_id
    goal.target_pose.header.stamp=rospy.Time.now()
    goal.target_pose.pose.position.x=goal_x
    goal.target_pose.pose.position.y=goal_y
    q=quaternion_from_euler(0,0,1.5707)
    goal.target_pose.pose.orientation=q

    client.send_goal(goal)
    wait=client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_goal_status_text()

if __name__ =='__main__':
    paths=readPaths()

    rospy.init_node('pub_path')
    mode_pub=rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=1)

    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
        processPaths(paths)
        rate.sleep()