import subprocess
# from turtle import pos
import rospy
from skimage import io
import numpy as np
import matplotlib.pyplot as plt
from subprocess import call
import os
import tensorflow.compat.v1 as tf
from skimage.transform import resize
import random
import numpy as np
from collections import deque
from tf_networks import create_CNN
from tensorboardX import SummaryWriter
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped
import cv2
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from copy import copy
import mean_shift as ms


# select mode
TRAIN = True
PLOT = True

# training environment parameters
ACTIONS = 81  # number of valid actions
GAMMA = 0.99  # decay rate of past observations
OBSERVE = 2e4  # timesteps to observe before training
EXPLORE = 2e6  # frames over which to anneal epsilon
epsilonEXPLORE = 60
# epsilonEXPLORE = 3
REPLAY_MEMORY = 10000  # number of previous transitions to remember
BATCH = 64  # size of minibatch
FINAL_RATE = 0  # final value of dropout rate
INITIAL_RATE = 0.9  # initial value of dropout rate
TARGET_UPDATE = 500  # update frequency of the target network
TAU = 0.05
# action_space = np.genfromtxt('/home/da/DRL_robot_exploration/scripts' + '/action_points.csv', delimiter=",")/20
# action_space = np.genfromtxt('/home/huangjie/DRL_robot_exploration/scripts' + '/action_points.csv', delimiter=",")/20
action_space = np.loadtxt('/home/huangjie/DRL_robot_exploration/scripts' + '/action_pi.txt')/20
network_dir = "../saved_networks/" + "cnn_" + str(ACTIONS)
if not os.path.exists(network_dir):
    os.makedirs(network_dir)
if TRAIN:
    log_dir = "../log/" + "cnn_" + str(ACTIONS)
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

local_size = 120
rob_size = 6
t1, t2 = np.meshgrid(np.linspace(-rob_size, rob_size, 2*rob_size + 1), np.linspace(-rob_size, rob_size, 2*rob_size + 1))
points = np.int32(np.vstack([t1.T.ravel(), t2.T.ravel()]).T)
idx = np.nonzero(np.linalg.norm(points,axis=1, keepdims=True) <= rob_size)
idx = np.asarray(idx[0]).ravel()
inrange_points = points[idx, :]


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

class launch_demo:
    def __init__(self, cmd=None):
        self.cmd = cmd
        self.interation = 1

    def launch(self, map):

        # self.child = subprocess.Popen(["roslaunch", "turtlebot3_gazebo", "turtlebot3_stage_"+str(self.interation)+".launch"])
        # self.interation += 1

        global_map = (io.imread("/home/da/STDR/src/stdr_resources/maps/train/"+map, 1) * 255).astype(int)
        robot_location = np.nonzero(global_map == 208)
        robot_location = np.array([np.array(robot_location)[1, 127], global_map.shape[0]-np.array(robot_location)[0, 127]])*0.05

        self.freegrid = sum(sum(global_map > 150))
        # print(f"\033[0;31;42mfreegrid:{self.freegrid}\033[0m")65280
        filedata = ''
        with open("/home/da/STDR/src/stdr_resources/maps/sparse_obstacles.yaml","r",encoding="utf-8") as f:
            flag = True
            for line in f:
                if flag:
                    line = "image: train/"+map+"\n"
                    # line = "image: train/"+str(self.interation)+".png\n"
                    flag = False
                    self.interation += 1
                filedata += line
        with open("/home/da/STDR/src/stdr_resources/maps/sparse_obstacles.yaml","w",encoding="utf-8") as f:
            f.write(filedata)
        self.child = subprocess.Popen(self.cmd)

        command = 'rosrun stdr_robot robot_handler add /home/da/STDR/src/stdr_resources/resources/robots/pandora_robot.yaml '+\
            str(robot_location[0])+' '+str(robot_location[1])+' '+'0'
        call(command, shell=True)
        rospy.sleep(3)
        return True

    def shutdown(self):
        self.child.terminate()
        self.child.wait()
        rospy.sleep(3)
        return True

def copy_weights(sess):
    trainable = tf.trainable_variables()
    for i in range(len(trainable)//2):
        assign_op = trainable[i+len(trainable)//2].assign(trainable[i] * TAU + trainable[i+len(trainable)//2] * (1-TAU))
        sess.run(assign_op)

def mapCallback(msg, position_x, position_y):
    # rospy.loginfo("1")
    data = np.array(msg.data)
    width = msg.info.width
    height = msg.info.height
    resolution, origin_x, origin_y = msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y
    map = []
    for i in range(height):
        map.append(data[i*width:(i+1)*width])
    map = np.array(map)
    x, y = int((position_x - origin_x)/resolution), int((position_y - origin_y)/resolution)
    minX = x - local_size
    maxX = x + local_size
    minY = y - local_size
    maxY = y + local_size

    if minX < 0:
        maxX = abs(minX) + maxX
        minX = 0
    if maxX > map.shape[1]:
        minX = minX - (maxX - map.shape[1])
        maxX = map.shape[1]
    if minY < 0:
        maxY = abs(minY) + maxY
        minY = 0
    if maxY > map.shape[0]:
        minY = minY - (maxY - map.shape[0])
        maxY = map.shape[0]

    Map = map[minY:maxY][:, minX:maxX] 

    map1 = (Map==100).astype(np.int)
    map2 = (Map==-1).astype(np.int)*0.5
    m = np.maximum(map1,map2)
    m1 = np.sign(np.add(np.sign(np.subtract(m, 0.6)),1))
    kernel = np.ones((5,5), np.uint8)
    m1 = cv2.dilate(m1, kernel)
    m = np.maximum(m,m1)

    Map = (m==0.5)*127 + (m==0)*255
    x -= minX
    y -= minY

    for i,j in zip(inrange_points[:,0],inrange_points[:,1]):
        map[y+j,x+i] = 76

    return map,Map

if __name__ == "__main__":
    # call('killall gzserver', shell=True)
    rospy.init_node('launch_demo1',anonymous=True)


    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    sess = tf.InteractiveSession(config=config)
    s, readout, keep_rate = create_CNN(ACTIONS)
    s_target, readout_target, keep_rate_target = create_CNN(ACTIONS)

    # define the cost function
    a = tf.placeholder("float", [None, ACTIONS])
    y = tf.placeholder("float", [None])
    readout_action = tf.reduce_sum(
        tf.multiply(readout, a), reduction_indices=1)
    cost = tf.reduce_mean(tf.square(y - readout_action))
    train_step = tf.train.AdamOptimizer(1e-5).minimize(cost)

    Step = 0
    drop_rate = INITIAL_RATE
    total_reward = np.empty([0, 0])
    finish_all_map = False

    # store the previous observations in replay memory
    D = deque()

    # tensorboard
    if TRAIN:
        writer = SummaryWriter(log_dir=log_dir)

    # saving and loading networks
    saver = tf.train.Saver()
    sess.run(tf.global_variables_initializer())
    copy_weights(sess)
    if not False:
        checkpoint = tf.train.get_checkpoint_state(network_dir)
        if checkpoint and checkpoint.model_checkpoint_path:
            saver.restore(sess, checkpoint.model_checkpoint_path)
            print("Successfully loaded:", checkpoint.model_checkpoint_path)
        else:
            print("Could not find old network weights")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.orientation.w = 1.0
    pub = rospy.Publisher('/Goal', Marker, queue_size=1) 

    while Step < EXPLORE:
        for i in range(10000):        
            # launch_nav.launch(map_list[i])
            step_t = 0
            a_t_coll = []

            while TRAIN and step_t <= epsilonEXPLORE:
                client = actionlib.SimpleActionClient('/move_base',MoveBaseAction)
                client.wait_for_server()
                Msg = rospy.wait_for_message('/odom', Odometry, timeout=5)
                msg = rospy.wait_for_message('/map', OccupancyGrid, timeout=5)
                position_x, position_y = Msg.pose.pose.position.x, Msg.pose.pose.position.y
                statemap, x_t = mapCallback(msg, position_x, position_y)
                x_t = resize(x_t*1.0, (84, 84))
                s_t = np.reshape(x_t, (1, 84, 84, 1))
                # scale down dropout rate
                if drop_rate > FINAL_RATE and Step > OBSERVE:
                    drop_rate -= (INITIAL_RATE - FINAL_RATE) / EXPLORE
                drop_rate = 0

                # choose an action by uncertainty
                readout_t = readout.eval(feed_dict={s: s_t, keep_rate: 1-drop_rate})[0]
                readout_t[a_t_coll] = None
                a_t = np.zeros([ACTIONS])
                action_index = np.nanargmax(readout_t)
                a_t[action_index] = 1
                # print(action_space[action_index])

                goal_x,goal_y = position_x + action_space[action_index][0], position_y + action_space[action_index][1]
                # goal_x,goal_y = position_x + 1 , position_y 
                goal.target_pose.pose.position.x = goal_x
                goal.target_pose.pose.position.y = goal_y
                # print(position_x,position_y)
                # print(goal_x,goal_y)

                points = Marker()       
                points.header.frame_id = msg.header.frame_id
                points.header.stamp = rospy.Time(0)
                points.ns = "markers2"
                points.id = 0
                points.type = Marker.POINTS
                points.action = Marker.ADD
                points.pose.orientation.w = 1.0
                # points.scale.x = 0.20
                # points.scale.y = 0.20
                points.scale.x = 0.10
                points.scale.y = 0.10
                points.color.r = 255.0/255.0
                points.color.g = 255.0/255.0
                points.color.b = 0.0/255.0
                points.color.a = 1
                points.lifetime = rospy.Duration()
                p = Point()
                p.x = goal_x
                p.y = goal_y
                p.z = 0
                pp = []
                pp.append(copy(p))     
                p.x = position_x
                p.y = position_y
                pp.append(copy(p))     
                points.points = pp
                pub.publish(points)

                # plt.cla()
                # plt.imshow(x_t,cmap='gray',vmin=0,vmax=255)
                # plt.plot((42,42+action_space[action_index][0]*20/2.86),(42,42+action_space[action_index][1]*20/2.86))
                # plt.scatter(42,42)
                # plt.pause(1)
                goal.target_pose.header.stamp = rospy.Time.now()
                client.send_goal(goal)
                for j in range(6):
                    wait = client.wait_for_result(rospy.Duration(1))
                    Msg = rospy.wait_for_message('/odom', Odometry, timeout=5)
                    position_x, position_y = Msg.pose.pose.position.x, Msg.pose.pose.position.y
                    if np.hypot(position_x - goal.target_pose.pose.position.x, position_y - goal.target_pose.pose.position.y)<=0.30:
                        break
                if not wait:
                    client.cancel_all_goals()
                    # print(f"\033[0;31;42mwait:{wait}\033[0m")

                Msg = rospy.wait_for_message('/odom', Odometry, timeout=5)
                msg = rospy.wait_for_message('/map', OccupancyGrid, timeout=5)
                position_x, position_y = Msg.pose.pose.position.x, Msg.pose.pose.position.y
                statemap_, x_t1 = mapCallback(msg, position_x, position_y)
                x_t1 = resize(x_t1*1.0, (84, 84))
                s_t1 = np.reshape(x_t1, (1, 84, 84, 1))
                reward = (sum(sum((statemap_ == 0)*1.0)) - sum(sum((statemap == 0)*1.0)))/1000

                if reward > 1:
                    reward = 1
                if np.hypot(position_x - goal_x, position_y - goal_y)<=1.00:
                    # print("\033[0;31;42mReach Goal\033[0m")
                    wait = False
                    if reward<0.05:
                        reward = -0.5
                        wait = True
                else:
                    # print("\033[0;31;42mCan't Reach Goal\033[0m")
                    reward = -1
                    wait = True

                # print(f"\033[0;31;42mreward:{reward}  RATE:{sum(sum((statemap_ == 0)*1.0))}||{launch_nav.freegrid}\033[0m")
                # print(f"\033[0;31;42mstategrid:{sum(sum((statemap_ == 0)*1.0))}    freegrid:{launch_nav.freegrid}\033[0m")
                print(f"\033[0;31;42mposition:{float('{0:.2f}'.format(position_x))},{float('{0:.2f}'.format(position_y))}\
                        goal:{float('{0:.2f}'.format(goal_x))},{float('{0:.2f}'.format(goal_y))}\033[0m")

                # if sum(sum((statemap_ == 0)*1.0))/launch_nav.freegrid>=0.55:
                #     break
                # continue
                # plt.cla()
                # plt.imshow(x_t,cmap='gray',vmin=0,vmax=255)
                # plt.plot((42,42+action_space[action_index][0]*20/3),(42,42+action_space[action_index][1]*20/3))
                # plt.pause(1)

                # # store the transition
                # D.append((s_t, a_t, reward, s_t1, wait))
                # if len(D) > REPLAY_MEMORY:
                #     D.popleft()

                # if Step > OBSERVE:
                #     # update target network
                #     if Step % TARGET_UPDATE == 0:
                #         copy_weights(sess)

                #     # sample a minibatch to train on
                #     minibatch = np.array(random.sample(D, BATCH))

                #     # get the batch variables
                #     s_j_batch = np.vstack(minibatch[:, 0])
                #     a_batch = np.vstack(minibatch[:, 1])
                #     r_batch = np.vstack(minibatch[:, 2]).flatten()
                #     s_j1_batch = np.vstack(minibatch[:, 3])

                #     readout_j1_batch = readout_target.eval(feed_dict={s_target: s_j1_batch, keep_rate_target: 1})
                #     end_multiplier = -(np.vstack(minibatch[:, 4]).flatten() - 1)
                #     y_batch = r_batch + GAMMA * np.max(readout_j1_batch) * end_multiplier

                #     # perform gradient step
                #     train_step.run(feed_dict={
                #         y: y_batch,
                #         a: a_batch,
                #         s: s_j_batch,
                #         keep_rate: 0.2}
                #     )

                #     # update tensorboard
                #     new_average_reward = np.average(total_reward[len(total_reward) - 10000:])
                #     writer.add_scalar('average reward', new_average_reward, step_t)

                step_t += 1
                Step += 1
                # total_reward = np.append(total_reward, r_t)

                # save progress
                # if Step%1e4 == 0:
                #     saver.save(sess, network_dir + '/cnn', global_step=Step)

                print("STEP", Step,"EpsilonSTEP", step_t, "/ DROPOUT", drop_rate, "/ ACTION", action_index, "/ REWARD", round(reward,5), "/ Terminal", wait, "\n")

                # reset the environment
                if reward<0:
                    msg = rospy.wait_for_message('/map', OccupancyGrid, timeout=5)
                    Msg = rospy.wait_for_message('/odom', Odometry, timeout=5)
                    position_x, position_y = Msg.pose.pose.position.x, Msg.pose.pose.position.y
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
                    m = map

                    m1 = np.sign(np.add(np.sign(np.subtract(m, 0.6)),1))
                    kernel = np.ones((10,10), np.uint8)
                    m1 = cv2.dilate(m1, kernel)
                    m2 = np.sign(np.add(np.sign(np.subtract(m, 0.4)),1))
                    kernel = np.ones((3,3), np.uint8)
                    m2 = cv2.erode(m2, kernel, iterations=1)
                    m2 = cv2.dilate(m2, kernel)
                    m = m1 + m2 * 0.5

                    temp = Frointer(m)
                    clust = Cluster(temp)
                    clust_len = len(clust)

                    Clust = []

                    mean_shifter = ms.MeanShift()
                    for i in range(clust_len):
                        if len(clust[i])<15:
                            continue
                        mean_shift_result = mean_shifter.cluster(np.array(clust[i])/20, kernel_bandwidth = 0.5)
                        cluster_assignments = mean_shift_result.cluster_ids
                        for j in range(np.unique(cluster_assignments).size):
                            ind = np.argwhere(cluster_assignments == j)
                            xx,yy=0.0,0.0
                            for k in ind:
                                xx +=clust[i][k[0]][0]
                                yy +=clust[i][k[0]][1]
                            xx /=len(ind)
                            yy /=len(ind)
                            Clust.append([int(yy)*0.05+msg.info.origin.position.x,\
                                 msg.info.origin.position.y+int(xx)*0.05])
                            
                    minDis = 1e20
                    if len(Clust) != 0:
                        # print(data.points)
                        for i in range(len(Clust)):
                            if minDis>np.hypot(position_x-Clust[i][0],position_y-Clust[i][1]):
                                minDis = np.hypot(position_x-Clust[i][0],position_y-Clust[i][1])
                                goal.target_pose.pose.position.x = Clust[i][0]
                                goal.target_pose.pose.position.y = Clust[i][1]
                    else:
                        print("\033[0;0;42mNext Map\033[0m")


                    points = Marker()       
                    points.header.frame_id = msg.header.frame_id
                    points.header.stamp = rospy.Time(0)
                    points.ns = "markers2"
                    points.id = 0
                    points.type = Marker.POINTS
                    points.action = Marker.ADD
                    points.pose.orientation.w = 1.0
                    # points.scale.x = 0.20
                    # points.scale.y = 0.20
                    points.scale.x = 0.10
                    points.scale.y = 0.10
                    points.color.r = 255.0/255.0
                    points.color.g = 255.0/255.0
                    points.color.b = 0.0/255.0
                    points.color.a = 1
                    points.lifetime = rospy.Duration()
                    p = Point()
                    p.x = goal.target_pose.pose.position.x
                    p.y = goal.target_pose.pose.position.y
                    p.z = 0
                    pp = []
                    pp.append(copy(p))  
                    points.points = pp
                    pub.publish(points)

                    goal.target_pose.header.stamp = rospy.Time.now()
                    client.send_goal(goal)
                    
                    poslist_x = []
                    poslist_y = []
                    for j in range(60):
                        # print(goal.target_pose.pose.position.x,goal.target_pose.pose.position.y)
                        wait = client.wait_for_result(rospy.Duration(0.2))
                        Msg = rospy.wait_for_message('/odom', Odometry, timeout=5)
                        position_x, position_y = Msg.pose.pose.position.x, Msg.pose.pose.position.y
                        # print(position_x, position_y)
                        if np.hypot(position_x - goal.target_pose.pose.position.x, position_y - goal.target_pose.pose.position.y)<=0.50:
                            break
                        poslist_x.append(float('{0:.1f}'.format(position_x)))
                        poslist_y.append(float('{0:.1f}'.format(position_y)))
                    if not wait:
                        client.cancel_all_goals()
                    if len(poslist_x) > 1 and len(set(poslist_x)) == 1 and len(set(poslist_y)) == 1:
                        print("\033[0;31;42mRobot STOP\033[0m")
                        break
                    print("\033[0;31;42mFroint Rescuer\033[0m")

            # launch_nav.shutdown()
# rosrun stdr_robot robot_handler replace /robot0 2 2 0
# rostopic pub /robot0/cmd_vel geometry_msgs/Twist -- '[-1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
# rospy.sleep(10)