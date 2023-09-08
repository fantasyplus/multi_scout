# opt_count相同时选次优，只考虑单步目标位置，如果自身在转圈或（与其余robot相距小于2或与其余分配点相同则重置，重置的基本原则是选择被访问次数少的自由坐标点，这比随机原则效果好）
# 解决自转圈、小车冲突问题√
import random
import time
import matplotlib.pyplot as plt
import numpy as np
import operator
import math
import xlwt
from cmath import inf
import time
import numpy as np
from scipy.optimize import linear_sum_assignment
import numpy as np
from copy import copy
import time
import matplotlib.pyplot as plt

#判断两车是否离得近
def xiangLin(x1, y1, x2, y2):
    dltax = x1 - x2
    dltay = y1 - y2
    flag = 0
    if dltax > 0:
        for i in range(x2, x1 + 1):
            if [i, y2] in obs_pos:
                flag = 1
            if [i, y1] in obs_pos:
                flag = 1
    if dltax < 0:
        for i in range(x1, x2 + 1):
            if [i, y1] in obs_pos:
                flag = 1
            if [i, y2] in obs_pos:
                flag = 1
    if dltay > 0:
        for i in range(y2, y1 + 1):
            if [x1, i] in obs_pos:
                flag = 1
            if [x2, i] in obs_pos:
                flag = 1
    if dltay < 0:
        for i in range(y1, y2 + 1):
            if [x1, i] in obs_pos:
                flag = 1
            if [x2, i] in obs_pos:
                flag = 1

    if (x1 - x2) == 0 and abs(y1 - y2) == xltj:
        if flag == 1:
            return False
        else:
            return True
    elif abs(x1 - x2) == xltj and (y1 - y2) == 0:
        if flag == 1:
            return False
        else:
            return True
    else:
        if abs(x1 - x2) + abs(y1 - y2) <= xltj:
            if flag == 1:
                return False
            else:
                return True


#计算小车当前坐标点的相邻自由坐标点
def nebourDot(x, y):
    global Action, obs_pos

    free_grid = []
    crr = [[x + 1, y], [x, y + 1], [x - 1, y], [x, y - 1]]
    for drr in crr:
        if drr not in Action:
            crr.remove(drr)
    free_grid = crr
    for arr in free_grid:
        if arr in obs_pos:
            free_grid.remove(arr)
    return free_grid

#判断小车前进方向
def fang_xiang(x1, y1, x2, y2):
    deltx = x1 - x2
    delty = y1 - y2
    f_x = []
    if deltx > 0:
        f_x = [0, 0, 0, 1]
    elif deltx < 0:
        f_x = [0, 0, 1, 0]
    elif delty > 0:
        f_x = [1, 0, 0, 0]
    elif delty < 0:
        f_x = [0, 1, 0, 0]
    return f_x


#求相邻自由坐标点中信息素最大的
def Max_va(free):
    h = []
    for [x, y] in free:
        h.append(Va[x][y])
    c = 0
    for j in range(len(h)):
        if h[j] == max(h):
            c = j
    return free[c]

#求相邻自由坐标点中被访问次数最少的
def Min_dotfre(free):
    h = []
    for [x, y] in free:
        for i in range(len(Action)):
            if operator.eq(Action[i], [x, y]):
                h.append(dot_fre[i])
    c = 0
    for j in range(len(h)):
        if h[j] == min(h):
            c = j
    return free[c]

#两车相遇时远离对方
def yuanlidf(x1, y1, x2, y2):
    h = []
    fre1 = nebourDot(x1, y1)
    fre2 = nebourDot(x2, y2)

    for i in range(len(fre1)):
        j = []
        h.append(j)
        for l in range(len(fre2)):
            j.append(abs(fre1[i][0] - fre2[l][0]) + abs(fre1[i][1] - fre2[l][1]))
    maxf = 0
    for i in range(len(fre1)):
        for j in range(len(fre2)):
            if h[i][j] > maxf:
                maxf = h[i][j]
    ij = []
    for i in range(len(fre1)):
        for j in range(len(fre2)):
            if h[i][j] == maxf:
                ij.append([i, j])
    f1 = []
    f2 = []
    for [i, j] in ij:
        f1.append(fre1[i])
        f2.append(fre2[j])
    aa = Max_va(f1)
    bb = Max_va(f2)
    if Va[aa[0]][aa[1]] > Va[bb[0]][bb[1]]:
        for [i, j] in ij:
            if operator.eq(aa, fre1[i]):
                bb = fre2[j]
    else:
        for [i, j] in ij:
            if operator.eq(bb, fre2[j]):
                aa = fre1[i]

    if operator.eq(aa, bb) or (operator.eq(aa, [x2, y2]) and operator.eq(bb, [x1, y1])):
        if len(ij) > 1:
            for [i, j] in ij:
                if operator.eq(aa, fre1[i]) and operator.eq(bb, fre2[j]):
                    ij.remove([i, j])
                    aa, bb = fre1[ij[0][0]], fre2[ij[0][1]]
        else:
            if len(fre1) > len(fre2):
                fre1.remove(aa)
                aa = Max_va(fre1)
            else:
                if len(fre2) == 1:
                    bb = fre2[0]
                else:
                    fre2.remove(bb)
                    bb = Max_va(fre2)

    return aa, bb


# 为小车分配目标位置
def update_carpos():
    global Cell, opt_count, car_pos, obs_pos, path_single, Action, distance_all, Va, w, tm
    # 根据小车的当前位置得出相邻自由坐标点
    free_gird = [[], []]
    lister = path_single[0]
    L2 = len(lister)
    for k in range(2):
        i = car_pos[k][0]
        j = car_pos[k][1]
        free_gird[k] = nebourDot(i, j)
    max_pos = [[], []] #存放临时分配的目标位置
    # 若两车相邻则根据相应的规则选择坐标点，放入max_pos
    if xiangLin(car_pos[0][0], car_pos[0][1], car_pos[1][0], car_pos[1][1]):
        max_pos[0], max_pos[1] = yuanlidf(car_pos[0][0], car_pos[0][1], car_pos[1][0], car_pos[1][1])
    else:
        L1 = len(car_pos)
        for i in range(L1):
            x0 = opt_count[i, 0, 0]
            y0 = opt_count[i, 0, 1]
            if len(free_gird[i]) > 1:
                for arr in free_gird[i]:
                    if operator.eq(arr, path_single[i][L2 - 2]):
                        free_gird[i].remove(arr)
            max_pos[i] = Max_va(free_gird[i])
    # 小车陷入矩形循环（沿一个矩形转圈）时，通过相应的规则跳出这个死区
    while (w > 10):
        for k in range(2):
            jishu = 0
            if w == 0:
                f_x1 = [1, 0, 0, 0]
            else:
                f_x1 = fang_xiang(car_pos[k][0], car_pos[k][1], path_single[k][len(path_single[k]) - 2][0],
                                  path_single[k][len(path_single[k]) - 2][1])
        for k in range(2):
            l = len(path_single[k])
            i = max_pos[k][0]
            j = max_pos[k][1]
            x = car_pos[k][0]
            y = car_pos[k][1]
            xunhuan1 = [[i, j + 1], [i - 1, j + 1], [i - 1, j]]
            xunhuan2 = [[i, j + 1], [i + 1, j + 1], [i + 1, j]]
            xunhuan3 = [[i, j - 1], [i + 1, j - 1], [i + 1, j]]
            xunhuan4 = [[i - 1, j], [i - 1, j - 1], [i, j - 1]]
            lujing = [path_single[k][l - 1], path_single[k][l - 2], path_single[k][l - 3]]
            shu = [0, 0, 0, 0]
            for arr in xunhuan1:
                if arr in lujing:
                    shu[0] += 1
            for arr in xunhuan2:
                if arr in lujing:
                    shu[1] += 1
            for arr in xunhuan3:
                if arr in lujing:
                    shu[2] += 1
            for arr in xunhuan4:
                if arr in lujing:
                    shu[3] += 1
            if shu[0] == 3:
                if operator.eq([i, j + 1], [x, y]):
                    if [i, j + 2] in Action and [i + 1, j + 1] in Action:
                        if Va[i][j + 2] < Va[i + 1][j + 1]:
                            max_pos[k] = [i + 1, j + 1]
                        else:
                            max_pos[k] = [i, j + 2]
                    elif [i, j + 2] in Action and [i + 1, j + 1] not in Action:
                        max_pos[k] = [i, j + 2]
                    elif [i, j + 2] not in Action and [i + 1, j + 1] in Action:
                        max_pos[k] = [i + 1, j + 1]
                    else:
                        c = [[i - 1, j + 1], [i, j]]
                        for [xx, yy] in c:
                            f_x2 = fang_xiang(xx, yy, car_pos[k][0], car_pos[k][1])
                            if operator.eq(f_x1, f_x2):
                                max_pos[k] = [xx, yy]
                                break
                            else:
                                jishu += 1
                        if jishu == len(c):
                            max_pos[k] = random.choice(c)
                if operator.eq([i - 1, j], [x, y]):
                    if [i - 2, j] in Action and [i - 1, j - 1] in Action:
                        if Va[i - 2][j] < Va[i - 1][j - 1]:
                            max_pos[k] = [i - 1, j - 1]
                        else:
                            max_pos[k] = [i - 2, j]
                    elif [i - 2, j] in Action and [i - 1, j - 1] not in Action:
                        max_pos[k] = [i - 2, j]
                    elif [i - 2, j] not in Action and [i - 1, j - 1] in Action:
                        max_pos[k] = [i - 1, j - 1]
                    else:
                        c = [[i - 1, j + 1], [i, j]]
                        for [xx, yy] in c:
                            f_x2 = fang_xiang(xx, yy, car_pos[k][0], car_pos[k][1])
                            if operator.eq(f_x1, f_x2):
                                max_pos[k] = [xx, yy]
                                break
                            else:
                                jishu += 1
                        if jishu == len(c):
                            max_pos[k] = random.choice(c)
            if shu[1] == 3:
                if operator.eq([i, j + 1], [x, y]):
                    if [i, j + 2] in Action and [i - 1, j + 1] in Action:
                        if Va[i][j + 2] < Va[i - 1][j + 1]:
                            max_pos[k] = [i - 1, j + 1]
                        else:
                            max_pos[k] = [i, j + 2]
                    elif [i, j + 2] in Action and [i - 1, j + 1] not in Action:
                        max_pos[k] = [i, j + 2]
                    elif [i, j + 2] not in Action and [i - 1, j + 1] in Action:
                        max_pos[k] = [i - 1, j + 1]
                    else:
                        c = [[i, j], [i + 1, j + 1]]
                        for [xx, yy] in c:
                            f_x2 = fang_xiang(xx, yy, car_pos[k][0], car_pos[k][1])
                            if operator.eq(f_x1, f_x2):
                                max_pos[k] = [xx, yy]
                                break
                            else:
                                jishu += 1
                        if jishu == len(c):
                            max_pos[k] = random.choice(c)
                if operator.eq([i + 1, j], [x, y]):
                    if [i + 2, j] in Action and [i + 1, j - 1] in Action:
                        if Va[i + 2][j] < Va[i + 1][j - 1]:
                            max_pos[k] = [i + 1, j - 1]
                        else:
                            max_pos[k] = [i + 2, j]
                    elif [i + 2, j] in Action and [i + 1, j - 1] not in Action:
                        max_pos[k] = [i + 2, j]
                    elif [i + 2, j] not in Action and [i + 1, j - 1] in Action:
                        max_pos[k] = [i + 1, j - 1]
                    else:
                        c = [[i, j], [i + 1, j + 1]]
                        for [xx, yy] in c:
                            f_x2 = fang_xiang(xx, yy, car_pos[k][0], car_pos[k][1])
                            if operator.eq(f_x1, f_x2):
                                max_pos[k] = [xx, yy]
                                break
                            else:
                                jishu += 1
                        if jishu == len(c):
                            max_pos[k] = random.choice(c)
            if shu[2] == 3:
                if operator.eq([i, j - 1], [x, y]):
                    if [i, j - 2] in Action and [i - 1, j - 1] in Action:
                        if Va[i][j - 2] < Va[i - 1][j - 1]:
                            max_pos[k] = [i - 1, j - 1]
                        else:
                            max_pos[k] = [i, j - 2]
                    elif [i, j - 2] in Action and [i - 1, j - 1] not in Action:
                        max_pos[k] = [i, j - 2]
                    elif [i, j - 2] not in Action and [i - 1, j - 1] in Action:
                        max_pos[k] = [i - 1, j - 1]
                    else:
                        c = [[i, j], [i + 1, j - 1]]
                        for [xx, yy] in c:
                            f_x2 = fang_xiang(xx, yy, car_pos[k][0], car_pos[k][1])
                            if operator.eq(f_x1, f_x2):
                                max_pos[k] = [xx, yy]
                                break
                            else:
                                jishu += 1
                        if jishu == len(c):
                            max_pos[k] = random.choice(c)
                if operator.eq([i + 1, j], [x, y]):
                    if [i + 2, j] in Action and [i + 1, j + 1] in Action:
                        if Va[i + 2][j] < Va[i + 1][j + 1]:
                            max_pos[k] = [i + 1, j + 1]
                        else:
                            max_pos[k] = [i + 2, j]
                    elif [i + 2, j] in Action and [i + 1, j + 1] not in Action:
                        max_pos[k] = [i + 2, j]
                    elif [i + 2, j] not in Action and [i + 1, j + 1] in Action:
                        max_pos[k] = [i + 1, j + 1]
                    else:
                        c = [[i, j], [i + 1, j - 1]]
                        for [xx, yy] in c:
                            f_x2 = fang_xiang(xx, yy, car_pos[k][0], car_pos[k][1])
                            if operator.eq(f_x1, f_x2):
                                max_pos[k] = [xx, yy]
                                break
                            else:
                                jishu += 1
                        if jishu == len(c):
                            max_pos[k] = random.choice(c)
            if shu[3] == 3:
                if operator.eq([i, j - 1], [x, y]):
                    if [i, j - 2] in Action and [i + 1, j - 1] in Action:
                        if Va[i][j - 2] < Va[i + 1][j - 1]:
                            max_pos[k] = [i + 1, j - 1]
                        else:
                            max_pos[k] = [i, j - 2]
                    elif [i, j - 2] in Action and [i + 1, j - 1] not in Action:
                        max_pos[k] = [i, j - 2]
                    elif [i, j - 2] not in Action and [i + 1, j - 1] in Action:
                        max_pos[k] = [i + 1, j - 1]
                    else:
                        c = [[i, j], [i - 1, j - 1]]
                        for [xx, yy] in c:
                            f_x2 = fang_xiang(xx, yy, car_pos[k][0], car_pos[k][1])
                            if operator.eq(f_x1, f_x2):
                                max_pos[k] = [xx, yy]
                                break
                            else:
                                jishu += 1
                        if jishu == len(c):
                            max_pos[k] = random.choice(c)
                if operator.eq([i - 1, j], [x, y]):
                    if [i - 2, j] in Action and [i - 1, j + 1] in Action:
                        if Va[i - 2][j] < Va[i - 1][j + 1]:
                            max_pos[k] = [i - 1, j + 1]
                        else:
                            max_pos[k] = [i - 2, j]
                    elif [i - 2, j] in Action and [i - 1, j + 1] not in Action:
                        max_pos[k] = [i - 2, j]
                    elif [i - 2, j] not in Action and [i - 1, j + 1] in Action:
                        max_pos[k] = [i - 1, j + 1]
                    else:
                        c = [[i, j], [i - 1, j - 1]]
                        for [xx, yy] in c:
                            f_x2 = fang_xiang(xx, yy, car_pos[k][0], car_pos[k][1])
                            if operator.eq(f_x1, f_x2):
                                max_pos[k] = [xx, yy]
                                break
                            else:
                                jishu += 1
                        if jishu == len(c):
                            max_pos[k] = random.choice(c)
        break
    # 小车沿L形轨迹来回振荡或在相邻两坐标点之间振荡时，通过相应的规则跳出死区
    for k in range(2):
        if w > 5 and operator.eq(max_pos[k], path_single[k][L2 - 2]) and operator.eq(car_pos[k], path_single[k][L2 - 3]):
            if path_single[k][L2 - 2] in free_gird[k]:
                free_gird[k].remove(path_single[k][L2 - 2])
            max_pos[k] = free_gird[k][0]

    for k in range(2):
        if w > 11 and operator.eq(path_single[k][L2 - 6], path_single[k][L2 - 2]) and operator.eq(path_single[k][L2 - 4], path_single[k][L2 - 8]) and operator.eq(car_pos[k], path_single[k][L2 - 5]):
            free_gird[k] = nebourDot(i, j)
            if len(free_gird[k]) > 1:
                for arr in free_gird[k]:
                    if arr in [path_single[k][L2 -2], path_single[k][L2-3], path_single[k][L2-4]]:
                        free_gird[k].remove(arr)
                max_pos[k] = Min_dotfre(free_gird[k])
    '''points = Marker()       
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

    p.x = 2
    p.y = 5
    pp.append(copy(p))   

    p.x = 5
    p.y = 2
    pp.append(copy(p))   
    # rospy.loginfo(len(pp))
    # rospy.loginfo(pp)
    points.points = pp
    pub.publish(points)'''
    car_pos = max_pos
    a1 = car_pos[0]
    b1 = car_pos[1]
    path_single[0].append(a1)
    path_single[1].append(b1)

#可视化
'''def display_on():
    plt.axis([0, h_1, 0, v_1])
    plt.xticks(xtick)
    plt.yticks(ytick)
    plt.plot([2, 10], [2, 2], color='black', linestyle='-')
    plt.plot([2, 2], [2, 14], color='black', linestyle='-')
    plt.plot([2, 6], [14, 14], color='black', linestyle='-')
    plt.plot([6, 6], [14, 11], color='black', linestyle='-')
    plt.plot([6, 8], [11, 11], color='black', linestyle='-')#
    plt.plot([8, 8], [10, 12], color='black', linestyle='-')#
    plt.plot([4, 4], [5, 6], color='black', linestyle='-')
    plt.plot([4, 12], [6, 6], color='black', linestyle='-')
    plt.plot([12, 12], [6, 7], color='black', linestyle='-')

    plt.plot([12, 15], [7, 7], color='black', linestyle='-')
    plt.plot([10, 10], [10, 11], color='black', linestyle='-')
    plt.plot([10, 13], [11, 11], color='black', linestyle='-')
    plt.plot([13, 13], [11, 10], color='black', linestyle='-')
    plt.plot([9, 9], [14, 17], color='black', linestyle='-')#
    plt.plot([9, 12], [14, 14], color='black', linestyle='-')
    plt.plot([11, 11], [13, 14], color='black', linestyle='-')
    plt.plot([11, 12], [13, 13], color='black', linestyle='-')
    plt.plot([12, 12], [14, 13], color='black', linestyle='-')
    plt.grid()


def display_off():
    plt.draw()
    plt.pause(0.01)
    plt.clf()


def show_car():
    global car_pos
    plt.plot(car_pos[0][0], car_pos[0][1], 'ro')
    plt.plot(car_pos[1][0], car_pos[1][1], 'y^')
    plt.plot(car_pos[2][0], car_pos[2][1], 'bs')


def show_pheromone():
    global Cell
    for i in range(h_1):
        for j in range(v_1):
            Cell[i][j] = round(Cell[i][j], 2)
            plt.text(i + 0.5, j + 0.5, Cell[i][j], color='k')


def show_path():
    global path_single
    l_1 = len(path_single[0])
    l_2 = len(path_single[1])
    l_3 = len(path_single[2])
    if l_1 < 5:
        for i in range(l_1 - 1):
            plt.plot([path_single[0][i][0], path_single[0][i + 1][0]], [path_single[0][i][1], path_single[0][i + 1][1]],
                     'r.-')
        for i in range(l_2 - 1):
            plt.plot([path_single[1][i][0], path_single[1][i + 1][0]], [path_single[1][i][1], path_single[1][i + 1][1]],
                     'y^-')
        for i in range(l_3 - 1):
            plt.plot([path_single[2][i][0], path_single[2][i + 1][0]], [path_single[2][i][1], path_single[2][i + 1][1]],
                     'bs-')
    else:
        for i in range(l_1 - 5, l_1 - 1):
            plt.plot([path_single[0][i][0], path_single[0][i + 1][0]], [path_single[0][i][1], path_single[0][i + 1][1]],
                     'r.-')
        for i in range(l_2 - 5, l_2 - 1):
            plt.plot([path_single[1][i][0], path_single[1][i + 1][0]], [path_single[1][i][1], path_single[1][i + 1][1]],
                     'y^-')
        for i in range(l_3 - 5, l_3 - 1):
            plt.plot([path_single[2][i][0], path_single[2][i + 1][0]], [path_single[2][i][1], path_single[2][i + 1][1]],
                     'bs-')'''

#根据朝向确定哪些网格被看到
def Whoisaw(k, x, y, flag):
    global Arrange, obs_pos, Cell
    if flag == [1, 0, 0, 0]:
        nebour = []
        Upmiddle = [[x - 1, y], [x + 1, y], [x - 1, y + 1], [x, y + 1], [x + 1, y + 1]]
        Upinflu = [[x - 1, y], [x - 2, y], [x, y], [x + 1, y], [x - 1, y + 1], [x - 2, y + 1], [x, y + 1],
                   [x + 1, y + 1]]
        for arr in Upinflu:
            if arr not in Arrange:
                Upinflu.remove(arr)
        for arr in Upmiddle:
            if arr in obs_pos:
                nebour.append(arr)

        if len(nebour) == 1:
            if nebour == [[x - 1, y + 1]]:  # nebour == [x, y + 1] or nebour == [x - 1, y] or nebour == [x + 1, y]
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
            if nebour == [[x + 1, y + 1]]:
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])

        elif len(nebour) == 2:
            if sorted(nebour) == sorted([[x - 1, y], [x - 1, y + 1]]):
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
                if [x - 2, y] in Upinflu:
                    Upinflu.remove([x - 2, y])
            if sorted(nebour) == sorted([[x + 1, y], [x + 1, y + 1]]):
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])
                if [x + 1, y] in Upinflu:
                    Upinflu.remove([x + 1, y])
            if sorted(nebour) == sorted([[x - 1, y + 1], [x, y + 1]]):
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
                if [x - 1, y + 1] in Upinflu:
                    Upinflu.remove([x - 1, y + 1])
            if sorted(nebour) == sorted([[x + 1, y + 1], [x, y + 1]]):
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])
                if [x, y + 1] in Upinflu:
                    Upinflu.remove([x, y + 1])
            if sorted(nebour) == sorted([[x - 1, y], [x + 1, y + 1]]):
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])
            if sorted(nebour) == sorted([[x - 1, y + 1], [x + 1, y + 1]]):
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])
            if sorted(nebour) == sorted([[x - 1, y + 1], [x + 1, y]]):
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])

        elif len(nebour) == 3:
            if sorted(nebour) == sorted([[x - 1, y], [x - 1, y + 1], [x, y + 1]]):
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
                if [x - 1, y + 1] in Upinflu:
                    Upinflu.remove([x - 1, y + 1])
                if [x - 2, y] in Upinflu:
                    Upinflu.remove([x - 2, y])
            elif sorted(nebour) == sorted([[x - 1, y + 1], [x, y + 1], [x + 1, y + 1]]):
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
                if [x - 1, y + 1] in Upinflu:
                    Upinflu.remove([x - 1, y + 1])
                if [x, y + 1] in Upinflu:
                    Upinflu.remove([x, y + 1])
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])
            elif sorted(nebour) == sorted([[x, y + 1], [x + 1, y + 1], [x + 1, y]]):
                if [x, y + 1] in Upinflu:
                    Upinflu.remove([x, y + 1])
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])
                if [x + 1, y] in Upinflu:
                    Upinflu.remove([x + 1, y])
            elif sorted(nebour) == sorted([[x - 1, y], [x, y + 1], [x + 1, y + 1]]):
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])
                if [x, y + 1] in Upinflu:
                    Upinflu.remove([x, y + 1])  # elif nebour == [[x - 1, y], [x, y + 1], [x + 1, y]]:
            elif sorted(nebour) == sorted([[x - 1, y], [x + 1, y + 1], [x + 1, y]]):
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])
                if [x + 1, y] in Upinflu:
                    Upinflu.remove([x + 1, y])
            elif sorted(nebour) == sorted([[x - 1, y + 1], [x + 1, y + 1], [x + 1, y]]):
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])
                if [x + 1, y] in Upinflu:
                    Upinflu.remove([x + 1, y])
            elif sorted(nebour) == sorted([[x - 1, y], [x - 1, y + 1], [x + 1, y + 1]]):
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
                if [x - 2, y] in Upinflu:
                    Upinflu.remove([x - 2, y])
            elif sorted(nebour) == sorted([[x - 1, y + 1], [x, y + 1], [x + 1, y]]):
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
                if [x - 1, y + 1] in Upinflu:
                    Upinflu.remove([x - 1, y + 1])
            else:  # nebour == [[x - 1, y], [x - 1, y + 1], [x + 1, y]]:
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
                if [x - 2, y] in Upinflu:
                    Upinflu.remove([x - 2, y])

        elif len(nebour) == 4:
            if sorted(nebour) == sorted([[x - 1, y], [x - 1, y + 1], [x, y + 1], [x + 1, y + 1]]):
                if [x - 2, y] in Upinflu:
                    Upinflu.remove([x - 2, y])
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
                if [x - 1, y + 1] in Upinflu:
                    Upinflu.remove([x - 1, y + 1])
                if [x, y + 1] in Upinflu:
                    Upinflu.remove([x, y + 1])
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])
            elif sorted(nebour) == sorted([[x - 1, y + 1], [x, y + 1], [x + 1, y + 1], [x + 1, y]]):
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
                if [x - 1, y + 1] in Upinflu:
                    Upinflu.remove([x - 1, y + 1])
                if [x, y + 1] in Upinflu:
                    Upinflu.remove([x, y + 1])
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])
                if [x + 1, y] in Upinflu:
                    Upinflu.remove([x + 1, y])
            elif sorted(nebour) == sorted([[x - 1, y], [x - 1, y + 1], [x, y + 1], [x + 1, y]]):
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
                if [x - 1, y + 1] in Upinflu:
                    Upinflu.remove([x - 1, y + 1])
                if [x - 2, y] in Upinflu:
                    Upinflu.remove([x - 2, y])
            elif sorted(nebour) == sorted([[x, y + 1], [x + 1, y + 1], [x + 1, y], [x - 1, y]]):
                if [x, y + 1] in Upinflu:
                    Upinflu.remove([x, y + 1])
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])
                if [x + 1, y] in Upinflu:
                    Upinflu.remove([x + 1, y])
            else:  # nebour == [[x - 1, y], [x - 1, y + 1], [x + 1, y], [x + 1, y + 1]]:
                if [x - 2, y] in Upinflu:
                    Upinflu.remove([x - 2, y])
                if [x - 2, y + 1] in Upinflu:
                    Upinflu.remove([x - 2, y + 1])
                if [x + 1, y] in Upinflu:
                    Upinflu.remove([x + 1, y])
                if [x + 1, y + 1] in Upinflu:
                    Upinflu.remove([x + 1, y + 1])

        elif len(nebour) == 5:
            if [x - 2, y] in Upinflu:
                Upinflu.remove([x - 2, y])
            if [x - 2, y + 1] in Upinflu:
                Upinflu.remove([x - 2, y + 1])
            if [x + 1, y] in Upinflu:
                Upinflu.remove([x + 1, y])
            if [x + 1, y + 1] in Upinflu:
                Upinflu.remove([x + 1, y + 1])
            if [x - 1, y + 1] in Upinflu:
                Upinflu.remove([x - 1, y + 1])
            if [x, y + 1] in Upinflu:
                Upinflu.remove([x, y + 1])
            '''for [i, j] in Upinflu:
                Cell[i][j] -= 5'''
        for arr in Upinflu:
            Visited[k].append(arr)
    if flag == [0, 1, 0, 0]:
        nebour = []
        Downmiddle = [[x - 1, y], [x - 1, y - 1], [x, y - 1], [x + 1, y - 1], [x + 1, y]]
        Downflu = [[x - 2, y - 1], [x - 1, y - 1], [x, y - 1], [x + 1, y - 1], [x - 2, y - 2], [x - 1, y - 2],
                   [x, y - 2], [x + 1, y - 2]]
        for brr in Downflu:
            if brr not in Arrange:
                Downflu.remove(brr)
        for brr in Downmiddle:
            if brr in obs_pos:
                nebour.append(brr)

        if len(nebour) == 1:
            if nebour == [[x - 1, y - 1]]:  # nebour == [x - 1, y] or nebour == [x, y - 1] or nebour == [x + 1, y]
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
            if nebour == [[x + 1, y - 1]]:
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])

        elif len(nebour) == 2:
            if sorted(nebour) == sorted([[x - 1, y], [x - 1, y - 1]]):
                if [x - 2, y - 1] in Downflu:
                    Downflu.remove([x - 2, y - 1])
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
            if sorted(nebour) == sorted([[x - 1, y - 1], [x, y - 1]]):
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Downflu:
                    Downflu.remove([x - 1, y - 2])
            if sorted(nebour) == sorted([[x, y - 1], [x + 1, y - 1]]):
                if [x, y - 2] in Downflu:
                    Downflu.remove([x, y - 2])
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])
            if sorted(nebour) == sorted([[x + 1, y - 1], [x + 1, y]]):
                if [x + 1, y - 1] in Downflu:
                    Downflu.remove([x + 1, y - 1])
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])
            if sorted(nebour) == sorted([[x - 1, y], [x + 1, y - 1]]):  # nebour == [[x - 1, y], [x, y - 1]] or nebour == [[x - 1, y], [x + 1, y]] or nebour == [[x, y - 1], [x + 1, y]]:
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])
            if sorted(nebour) == sorted([[x - 1, y - 1], [x + 1, y - 1]]):
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])
            if sorted(nebour) == sorted([[x - 1, y - 1], [x + 1, y]]):
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])

        elif len(nebour) == 3:
            if sorted(nebour) == sorted([[x - 1, y], [x - 1, y - 1], [x, y - 1]]):
                if [x - 2, y - 1] in Downflu:
                    Downflu.remove([x - 2, y - 1])
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Downflu:
                    Downflu.remove([x - 1, y - 2])
            if sorted(nebour) == sorted([[x - 1, y - 1], [x, y - 1], [x + 1, y - 1]]):
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Downflu:
                    Downflu.remove([x - 1, y - 2])
                if [x, y - 2] in Downflu:
                    Downflu.remove([x, y - 2])
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])
            if sorted(nebour) == sorted([[x, y - 1], [x + 1, y - 1], [x + 1, y]]):
                if [x, y - 2] in Downflu:
                    Downflu.remove([x, y - 2])
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])
                if [x + 1, y - 1] in Downflu:
                    Downflu.remove([x + 1, y - 1])
            if sorted(nebour) == sorted([[x - 1, y], [x, y - 1], [x + 1, y - 1]]):
                if [x, y - 2] in Downflu:
                    Downflu.remove([x, y - 2])
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])
            if sorted(nebour) == sorted([[x - 1, y], [x + 1, y - 1], [x + 1, y]]):
                if [x + 1, y - 1] in Downflu:
                    Downflu.remove([x + 1, y - 1])
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])
            if sorted(nebour) == sorted([[x - 1, y - 1], [x + 1, y - 1], [x + 1, y]]):
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
                if [x + 1, y - 1] in Downflu:
                    Downflu.remove([x + 1, y - 1])
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])
            if sorted(nebour) == sorted([[x - 1, y], [x - 1, y - 1], [x + 1, y - 1]]):
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])
                if [x - 2, y - 1] in Downflu:
                    Downflu.remove([x - 2, y - 1])
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
            if sorted(nebour) == sorted([[x - 1, y], [x - 1, y - 1], [x + 1, y]]):
                if [x - 2, y - 1] in Downflu:
                    Downflu.remove([x - 2, y - 1])
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
            if sorted(nebour) == sorted([[x - 1, y - 1], [x, y - 1], [x + 1, y]]):
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Downflu:
                    Downflu.remove([x - 1, y - 2])

        elif len(nebour) == 4:
            if sorted(nebour) == sorted([[x - 1, y], [x - 1, y - 1], [x, y - 1], [x + 1, y - 1]]):
                if [x - 2, y - 1] in Downflu:
                    Downflu.remove([x - 2, y - 1])
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Downflu:
                    Downflu.remove([x - 1, y - 2])
                if [x, y - 2] in Downflu:
                    Downflu.remove([x, y - 2])
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])
            elif sorted(nebour) == sorted([[x - 1, y - 1], [x, y - 1], [x + 1, y - 1], [x + 1, y]]):
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Downflu:
                    Downflu.remove([x - 1, y - 2])
                if [x, y - 2] in Downflu:
                    Downflu.remove([x, y - 2])
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])
                if [x + 1, y - 1] in Downflu:
                    Downflu.remove([x + 1, y - 1])
            elif sorted(nebour) == sorted([[x - 1, y], [x - 1, y - 1], [x, y - 1], [x + 1, y]]):
                if [x - 2, y - 1] in Downflu:
                    Downflu.remove([x - 2, y - 1])
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Downflu:
                    Downflu.remove([x - 1, y - 2])
            elif sorted(nebour) == sorted([[x - 1, y], [x, y - 1], [x + 1, y - 1], [x + 1, y]]):
                if [x, y - 2] in Downflu:
                    Downflu.remove([x, y - 2])
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])
                if [x + 1, y - 1] in Downflu:
                    Downflu.remove([x + 1, y - 1])
            else:
                if [x - 2, y - 1] in Downflu:
                    Downflu.remove([x - 2, y - 1])
                if [x - 2, y - 2] in Downflu:
                    Downflu.remove([x - 2, y - 2])
                if [x + 1, y - 1] in Downflu:
                    Downflu.remove([x + 1, y - 1])
                if [x + 1, y - 2] in Downflu:
                    Downflu.remove([x + 1, y - 2])

        elif len(nebour) == 5:
            if [x - 2, y - 1] in Downflu:
                Downflu.remove([x - 2, y - 1])
            if [x - 2, y - 2] in Downflu:
                Downflu.remove([x - 2, y - 2])
            if [x - 1, y - 2] in Downflu:
                Downflu.remove([x - 1, y - 2])
            if [x, y - 2] in Downflu:
                Downflu.remove([x, y - 2])
            if [x + 1, y - 2] in Downflu:
                Downflu.remove([x + 1, y - 2])
            if [x + 1, y - 1] in Downflu:
                Downflu.remove([x + 1, y - 1])
        for arr in Downflu:
            Visited[k].append(arr)
    if flag == [0, 0, 1, 0]:
        nebour = []
        Leftmiddle = [[x - 1, y], [x - 1, y - 1], [x - 1, y + 1], [x, y - 1], [x, y + 1]]
        Leftflu = [[x - 1, y], [x - 1, y + 1], [x - 1, y - 1], [x - 1, y - 2], [x - 2, y], [x - 2, y + 1],
                   [x - 2, y - 1], [x - 2, y - 2]]
        for crr in Leftflu:
            if crr not in Arrange:
                Leftflu.remove(crr)
        for crr in Leftmiddle:
            if crr in obs_pos:
                nebour.append(crr)

        if len(nebour) == 1:
            if nebour == [[x - 1, y - 1]]:
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
            if nebour == [[x - 1, y + 1]]:
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])

        elif len(nebour) == 2:
            if sorted(nebour) == sorted([[x, y - 1], [x - 1, y - 1]]):
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Leftflu:
                    Leftflu.remove([x - 1, y - 2])
            if sorted(nebour) == sorted([[x - 1, y - 1], [x - 1, y]]):
                if [x - 2, y - 1] in Leftflu:
                    Leftflu.remove([x - 2, y - 1])
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
            if sorted(nebour) == sorted([[x - 1, y], [x - 1, y + 1]]):
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
                if [x - 2, y] in Leftflu:
                    Leftflu.remove([x - 2, y])
            if sorted(nebour) == sorted([[x - 1, y + 1], [x, y + 1]]):
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
                if [x - 1, y + 1] in Leftflu:
                    Leftflu.remove([x - 1, y + 1])
            if sorted(nebour) == sorted([[x, y - 1], [x - 1, y + 1]]):
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
            if sorted(nebour) == sorted([[x - 1, y - 1], [x - 1, y + 1]]):
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
            if sorted(nebour) == sorted([[x - 1, y - 1], [x, y + 1]]):
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])

        elif len(nebour) == 3:
            if sorted(nebour) == sorted([[x, y - 1], [x - 1, y - 1], [x - 1, y]]):
                if [x - 2, y - 1] in Leftflu:
                    Leftflu.remove([x - 2, y - 1])
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Leftflu:
                    Leftflu.remove([x - 1, y - 2])
            if sorted(nebour) == sorted([[x - 1, y - 1], [x - 1, y], [x - 1, y + 1]]):
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
                if [x - 2, y - 1] in Leftflu:
                    Leftflu.remove([x - 2, y - 1])
                if [x - 2, y] in Leftflu:
                    Leftflu.remove([x - 2, y])
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
            if sorted(nebour) == sorted([[x - 1, y], [x - 1, y + 1], [x, y + 1]]):
                if [x - 1, y + 1] in Leftflu:
                    Leftflu.remove([x - 1, y + 1])
                if [x - 2, y] in Leftflu:
                    Leftflu.remove([x - 2, y])
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
            if sorted(nebour) == sorted([[x, y - 1], [x - 1, y - 1], [x - 1, y + 1]]):
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Leftflu:
                    Leftflu.remove([x - 1, y - 2])
            if sorted(nebour) == sorted([[x, y - 1], [x - 1, y - 1], [x, y + 1]]):
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Leftflu:
                    Leftflu.remove([x - 1, y - 2])
            if sorted(nebour) == sorted([[x - 1, y - 1], [x - 1, y], [x, y + 1]]):
                if [x - 2, y - 1] in Leftflu:
                    Leftflu.remove([x - 2, y - 1])
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
            if sorted(nebour) == sorted([[x - 1, y], [x - 1, y + 1], [x, y - 1]]):
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
                if [x - 2, y] in Leftflu:
                    Leftflu.remove([x - 2, y])
            if sorted(nebour) == sorted([[x - 1, y + 1], [x, y + 1], [x - 1, y - 1]]):
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
                if [x - 1, y + 1] in Leftflu:
                    Leftflu.remove([x - 1, y + 1])
            if sorted(nebour) == sorted([[x - 1, y + 1], [x, y + 1], [x, y - 1]]):
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
                if [x - 1, y + 1] in Leftflu:
                    Leftflu.remove([x - 1, y + 1])

        elif len(nebour) == 4:
            if sorted(nebour) == sorted([[x, y - 1], [x - 1, y - 1], [x - 1, y], [x - 1, y + 1]]):
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
                if [x - 2, y] in Leftflu:
                    Leftflu.remove([x - 2, y])
                if [x - 2, y - 1] in Leftflu:
                    Leftflu.remove([x - 2, y - 1])
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Leftflu:
                    Leftflu.remove([x - 1, y - 2])
            if sorted(nebour) == sorted([[x - 1, y - 1], [x - 1, y], [x - 1, y + 1], [x, y + 1]]):
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
                if [x - 2, y] in Leftflu:
                    Leftflu.remove([x - 2, y])
                if [x - 2, y - 1] in Leftflu:
                    Leftflu.remove([x - 2, y - 1])
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
                if [x - 1, y + 1] in Leftflu:
                    Leftflu.remove([x - 1, y + 1])
            if sorted(nebour) == sorted([[x, y - 1], [x - 1, y - 1], [x - 1, y], [x, y + 1]]):
                if [x - 2, y - 1] in Leftflu:
                    Leftflu.remove([x - 2, y - 1])
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Leftflu:
                    Leftflu.remove([x - 1, y - 2])
            if sorted(nebour) == sorted([[x - 1, y], [x - 1, y + 1], [x, y + 1], [x, y - 1]]):
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
                if [x - 2, y] in Leftflu:
                    Leftflu.remove([x - 2, y])
                if [x - 1, y + 1] in Leftflu:
                    Leftflu.remove([x - 1, y + 1])
            if sorted(nebour) == sorted([[x, y - 1], [x - 1, y - 1], [x - 1, y + 1], [x, y + 1]]):
                if [x - 2, y + 1] in Leftflu:
                    Leftflu.remove([x - 2, y + 1])
                if [x - 1, y + 1] in Leftflu:
                    Leftflu.remove([x - 1, y + 1])
                if [x - 2, y - 2] in Leftflu:
                    Leftflu.remove([x - 2, y - 2])
                if [x - 1, y - 2] in Leftflu:
                    Leftflu.remove([x - 1, y - 2])

        elif len(nebour) == 5:
            if [x - 2, y + 1] in Leftflu:
                Leftflu.remove([x - 2, y + 1])
            if [x - 2, y] in Leftflu:
                Leftflu.remove([x - 2, y])
            if [x - 2, y - 1] in Leftflu:
                Leftflu.remove([x - 2, y - 1])
            if [x - 2, y - 2] in Leftflu:
                Leftflu.remove([x - 2, y - 2])
            if [x - 1, y + 1] in Leftflu:
                Leftflu.remove([x - 1, y + 1])
            if [x - 1, y - 2] in Leftflu:
                Leftflu.remove([x - 1, y - 2])
        for arr in Leftflu:
            Visited[k].append(arr)
    if flag == [0, 0, 0, 1]:
        nebour = []
        Rightmiddle = [[x, y + 1], [x + 1, y + 1], [x + 1, y], [x + 1, y - 1], [x, y - 1]]
        Righflu = [[x, y + 1], [x, y], [x, y - 1], [x, y - 2], [x + 1, y + 1], [x + 1, y], [x + 1, y - 1],
                   [x + 1, y - 2]]
        for drr in Righflu:
            if drr not in Arrange:
                Righflu.remove(drr)
        for drr in Righflu:
            if drr[0] not in xxx:
                Righflu.remove(drr)
            if drr[1] not in yyy:
                Righflu.remove(drr)
        for drr in Rightmiddle:
            if drr in obs_pos:
                nebour.append(drr)

        if len(nebour) == 1:
            if nebour == [[x + 1, y + 1]]:
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
            if nebour == [[x + 1, y - 1]]:
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])

        elif len(nebour) == 2:
            if sorted(nebour) == sorted([[x, y - 1], [x + 1, y - 1]]):
                if [x, y - 2] in Righflu:
                    Righflu.remove([x, y - 2])
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])
            if sorted(nebour) == sorted([[x + 1, y - 1], [x + 1, y]]):
                if [x + 1, y - 1] in Righflu:
                    Righflu.remove([x + 1, y - 1])
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])
            if sorted(nebour) == sorted([[x + 1, y], [x + 1, y + 1]]):
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
                if [x + 1, y] in Righflu:
                    Righflu.remove([x + 1, y])
            if sorted(nebour) == sorted([[x + 1, y + 1], [x, y + 1]]):
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
                if [x, y + 1] in Righflu:
                    Righflu.remove([x, y + 1])
            if sorted(nebour) == sorted([[x, y - 1], [x + 1, y + 1]]):
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
            if sorted(nebour) == sorted([[x + 1, y - 1], [x + 1, y + 1]]):
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
            if sorted(nebour) == sorted([[x + 1, y - 1], [x, y + 1]]):
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])

        elif len(nebour) == 3:
            if sorted(nebour) == sorted([[x, y - 1], [x + 1, y - 1], [x + 1, y]]):
                if [x, y - 2] in Righflu:
                    Righflu.remove([x, y - 2])
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])
                if [x + 1, y - 1] in Righflu:
                    Righflu.remove([x + 1, y - 1])
            if sorted(nebour) == sorted([[x + 1, y - 1], [x + 1, y], [x + 1, y + 1]]):
                if [x + 1, y - 1] in Righflu:
                    Righflu.remove([x + 1, y - 1])
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])
                if [x + 1, y] in Righflu:
                    Righflu.remove([x + 1, y])
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
            if sorted(nebour) == sorted([[x + 1, y], [x + 1, y + 1], [x, y + 1]]):
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
                if [x + 1, y] in Righflu:
                    Righflu.remove([x + 1, y])
                if [x, y + 1] in Righflu:
                    Righflu.remove([x, y + 1])
            if sorted(nebour) == sorted([[x, y - 1], [x + 1, y - 1], [x + 1, y + 1]]):
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
                if [x, y - 2] in Righflu:
                    Righflu.remove([x, y - 2])
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])
            if sorted(nebour) == sorted([[x, y - 1], [x + 1, y - 1], [x, y + 1]]):
                if [x, y - 2] in Righflu:
                    Righflu.remove([x, y - 2])
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])
            if sorted(nebour) == sorted([[x + 1, y - 1], [x + 1, y], [x, y + 1]]):
                if [x + 1, y - 1] in Righflu:
                    Righflu.remove([x + 1, y - 1])
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])
            if sorted(nebour) == sorted([[x + 1, y], [x + 1, y + 1], [x, y - 1]]):
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
                if [x + 1, y] in Righflu:
                    Righflu.remove([x + 1, y])
            if sorted(nebour) == sorted([[x, y + 1], [x + 1, y + 1], [x + 1, y - 1]]):
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
                if [x, y + 1] in Righflu:
                    Righflu.remove([x, y + 1])
            if sorted(nebour) == sorted([[x, y + 1], [x + 1, y + 1], [x, y - 1]]):
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
                if [x, y + 1] in Righflu:
                    Righflu.remove([x, y + 1])

        elif len(nebour) == 4:
            if sorted(nebour) == sorted([[x, y - 1], [x + 1, y - 1], [x + 1, y], [x + 1, y + 1]]):
                if [x, y - 2] in Righflu:
                    Righflu.remove([x, y - 2])
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])
                if [x + 1, y - 1] in Righflu:
                    Righflu.remove([x + 1, y - 1])
                if [x + 1, y] in Righflu:
                    Righflu.remove([x + 1, y])
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
            if sorted(nebour) == sorted([[x + 1, y - 1], [x + 1, y], [x + 1, y + 1], [x, y + 1]]):
                if [x, y + 1] in Righflu:
                    Righflu.remove([x, y + 1])
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])
                if [x + 1, y - 1] in Righflu:
                    Righflu.remove([x + 1, y - 1])
                if [x + 1, y] in Righflu:
                    Righflu.remove([x + 1, y])
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
            if sorted(nebour) == sorted([[x, y - 1], [x + 1, y - 1], [x + 1, y], [x, y + 1]]):
                if [x, y - 2] in Righflu:
                    Righflu.remove([x, y - 2])
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])
                if [x + 1, y - 1] in Righflu:
                    Righflu.remove([x + 1, y - 1])
            if sorted(nebour) == sorted([[x + 1, y], [x + 1, y + 1], [x, y + 1], [x, y - 1]]):
                if [x, y + 1] in Righflu:
                    Righflu.remove([x, y + 1])
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])
                if [x + 1, y] in Righflu:
                    Righflu.remove([x + 1, y])
            if sorted(nebour) == sorted([[x, y - 1], [x + 1, y - 1], [x + 1, y + 1], [x, y + 1]]):
                if [x, y - 2] in Righflu:
                    Righflu.remove([x, y - 2])
                if [x + 1, y - 2] in Righflu:
                    Righflu.remove([x + 1, y - 2])
                if [x, y + 1] in Righflu:
                    Righflu.remove([x, y + 1])
                if [x + 1, y + 1] in Righflu:
                    Righflu.remove([x + 1, y + 1])

        elif len(nebour) == 5:
            if [x, y - 2] in Righflu:
                Righflu.remove([x, y - 2])
            if [x + 1, y - 2] in Righflu:
                Righflu.remove([x + 1, y - 2])
            if [x, y + 1] in Righflu:
                Righflu.remove([x, y + 1])
            if [x + 1, y + 1] in Righflu:
                Righflu.remove([x + 1, y + 1])
            if [x + 1, y - 1] in Righflu:
                Righflu.remove([x + 1, y - 1])
            if [x + 1, y] in Righflu:
                Righflu.remove([x + 1, y])
        for arr in Righflu:
            Visited[k].append(arr)


#小车前进方向及所看到的网格确定函数
def update_cell(k):
    global car_pos, Cell, path_single
    flag = []
    x = car_pos[k][0]
    y = car_pos[k][1]
    pathlist = path_single[k]
    a0 = pathlist[len(pathlist) - 2][0]
    b0 = pathlist[len(pathlist) - 2][1]
    if (x - a0) > 0:
        flag = [0, 0, 0, 1]
    if (x - a0) < 0:
        flag = [0, 0, 1, 0]
    if (y - b0) > 0:
        flag = [1, 0, 0, 0]
    if (y - b0) < 0:
        flag = [0, 1, 0, 0]
    Whoisaw(k, x, y, flag)


#网格信息素衰减函数
def shuaijian(arr, k, x, y):
    for [i, j] in arr:
        dtx = x - i
        dty = y - j
        if [dtx, dty] in [[2, -1], [2, 2], [-1, -1], [-1, 2]]:
            Cell[i][j] *= 0.9
        elif [dtx, dty] in [[1, -1], [2, 0], [0, -1], [2, 1], [1, 2], [0, 2], [-1, 0], [-1, 1]]:
            Cell[i][j] *= 0.8
        else:
            Cell[i][j] *= 0.5


def Initial():
    global Cell, Arrange, Action, car_pos, path_single, obs_pos, Va, frequence
    for i in range(h_1):
        for j in range(v_1):
            Arrange.append([i, j])
    for i in range(h_1 - 1):
        for j in range(v_1 - 1):
            Action.append([i + 1, j + 1])
    for arr in obs_pos:
        if arr in Action:
            Action.remove(arr)

    for i in range(h_1):
        j = []
        Cell.append(j)
        for l in range(v_1):
            j.append(1)

    for i in range(len(Action)):
        dot_fre.append(0)
    for i in range(len(Action)):
        if operator.eq(Action[i], car_pos[0]) or operator.eq(Action[i], car_pos[1]):
            dot_fre[i] += 1

    for i in range(h_1):
        l = []
        Va.append(l)
        for j in range(v_1):
            l.append(-10000)
    for i in range(1, h_1):
        for j in range(1, v_1):
            if [i, j] not in obs_pos:
                Va[i][j] = (Cell[i - 1][j - 1] + Cell[i - 1][j] + Cell[i][j - 1] + Cell[i][j]) / 4

    for i in range(h_1):
        j = []
        frequence.append(j)
        for l in range(v_1):
            j.append(0)
    for i in range(h_1):
        j = []
        Time.append(j)
        for l in range(v_1):
            j.append(0)
    for i in range(h_1):
        j = []
        idl.append(j)
        for l in range(v_1):
            j.append(0)
    for i in range(h_1):
        j = []
        cell.append(j)
        for l in range(v_1):
            j.append(0)
    # 初次分配时小车看到的网格及其信息素更新、访问次数更新
    all_fang = [[[1, 0], [1, 1], [1, 2], [2, 0], [2, 1]], [[11, 14], [12, 14], [13, 14], [11, 13], [12, 13], [13,
13]]]
    for k in range(2):
        shuaijian(all_fang[k], k, car_pos[k][0], car_pos[k][1])
    for k in range(2):
        for [i, j] in all_fang[k]:
            frequence[i][j] = 1
'''def node():
	rospy.init_node('assignTarget', anonymous=False)
	# rate = rospy.Rate(1)
	rospy.Subscriber("/gazebo/link_states", LinkStates, odomCallback)
	pub = rospy.Publisher('/frointer', Marker, queue_size=1) 
	# rate.sleep()'''
# 初始化
Cell = []  # 网格的信息素值
Arrange = [] # 自由坐标点和墙体所占据坐标点集合
Action = []  # 自由坐标点集合
h_1, v_1 = 14, 16  # 算法所使用地图的大小
geshu = 223  # gazebo中红色网格个数

car_pos = [[1, 1], [13, 15]] #小车位置
path_single = [[[1, 1]], [[13, 15]]] #小车经过的自由坐标点
obs_pos = [[2, 2], [3, 2], [4, 2], [5, 2], [6, 2], [7, 2], [8, 2], [9, 2], [2, 3], [2, 4], [2, 5], [2, 6], [2, 7], [2, 8], [2, 9], [2, 10], [2, 11], [2, 12], [2, 13], [3, 13], [4, 13], [5, 13], [6, 13], [6, 12], [4, 5], [4, 6], [5, 6], [6, 6], [7, 6], [8, 6], [9, 6], [10, 6], [11, 6], [12, 7], [13, 7], [10, 10], [11, 13], [6, 11], [6, 10], [7, 10], [8, 10], [8, 11], [9, 10], [9, 9], [11, 10], [12, 10], [10, 12], [11, 12], [10, 13], [9, 13], [8, 13], [8, 14], [8, 15], [11, 7], [0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, 0], [7, 0], [8, 0], [9, 0], [10, 0], [11, 0], [12, 0], [13, 0], [14, 0], [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [0, 7], [0, 8], [0, 9], [0, 10], [0, 11], [0, 12], [0, 13], [0, 14], [0, 15], [0, 16], [1, 16], [2, 16], [3, 16], [4, 16], [5, 16], [6, 16], [7, 16], [8, 16], [9, 16], [10, 16], [11, 16], [12, 16], [13, 16], [14, 16], [14, 1], [14, 2], [14, 3], [14, 4], [14, 5], [14, 6], [14, 7], [14, 8], [14, 9], [14, 10], [14, 11], [14, 12], [14, 13], [14, 14], [14, 15], [14, 16]] #墙体占据的坐标点
frequence = []  # 网格被访问次数
Va = []  # 坐标点的信息素值，即坐标点周围四个网格的Cell平均值
dot_fre = [] # 坐标点被访问次数
Time = [] # 网格被看到时的时间点
idl = [] # 网格的平均空闲时间
idl_shuns_cell = [] # 每次分配的全局最大瞬时空闲时间
cell = [] # 网格的瞬时空闲时间
glob_cell_idl = [] # 每次分配的全局平均空闲时间
Initial()
CTU = [] # 总冲突次数
w = 0 # 分配次数
t = ? # 初始时间点

while (w != 1000):
    chongtu = 0 # 单次冲突次数
    update_carpos() # 获取小车当前坐标点
    # 统计自由坐标点被访问次数
    for i in range(len(Action)):
        if operator.eq(Action[i], car_pos[0]) or operator.eq(Action[i], car_pos[1]):
            dot_fre[i] += 1
    # 小车这次看到的网格及其信息素更新
    Visited = [[], []]
    for i in range(2):
        update_cell(i)
    for i in range(2):
        shuaijian(Visited[i], i, car_pos[i][0], car_pos[i][1])
    all_visit = []
    for k in range(2):
        for arr in Visited[k]:
            if arr in all_visit:
                continue
            else:
                all_visit.append(arr)
    #print(f'w:{w}')
    w += 1
    # 计算网格的相关参数
    for i in range(h_1):
        for j in range(v_1):
            if [i, j] in all_visit:
                Time[i][j] = t
            else:
                Cell[i][j] *= 1.1

    for i in range(1, h_1):
        for j in range(1, v_1):
            if [i, j] not in obs_pos:
                Va[i][j] = (Cell[i - 1][j - 1] + Cell[i - 1][j] + Cell[i][j - 1] + Cell[i][j]) / 4

    for i in range(h_1):
        for j in range(v_1):
            if [i, j] == [10, 12]:
                continue
            else:
                cell[i][j] = t - Time[i][j]
                idl[i][j] = (frequence[i][j] * idl[i][j] + cell[i][j]) / (frequence[i][j] + 1)
            # print([i, j])
    for [i, j] in all_visit:
        frequence[i][j] += 1
    shuns = 0
    for i in range(h_1):
        for j in range(v_1):
            if [i, j] == [10, 12]:
                continue
            if cell[i][j] > shuns:
                shuns = cell[i][j]
    idl_shuns_cell.append(shuns)
    s = 0
    for i in range(h_1):
        for j in range(v_1):
            if [i, j] == [10, 12]:
                continue
            else:
                s += idl[i][j]
    glob_cell_idl.append(round((s / geshu), 2))

    if operator.eq(path_single[0][len(path_single[0]) - 1], path_single[1][len(path_single[0]) - 1]):
        chongtu += 1
    if operator.eq(path_single[0][len(path_single[0]) - 2], car_pos[1]) and operator.eq(
            path_single[1][len(path_single[0]) - 2], car_pos[0]):
        chongtu += 1
    CTU.append(chongtu)
#输出Excel
# d = []
# for i in range(len(CTU)):
#     d.append(i + 1)
# book = xlwt.Workbook(encoding='utf-8', style_compression=0)
# sheet = book.add_sheet('CR', cell_overwrite_ok=True)
# col = ('行', '列')
# for i in range(0, 2):
#     sheet.write(0, i, col[i])
# datalist = zip(d, CTU)
# list_new = list(list(arr) for arr in datalist)
# for i in range(0, len(d)):
#     data = list_new[i]
#     for j in range(0, 2):
#         sheet.write(i + 1, j, data[j])
# savepath = '冲突.xls'
# book.save(savepath)

# d1 = []
# for i in range(w):
#     d1.append(i + 1)
# book1 = xlwt.Workbook(encoding='utf-8', style_compression=0)
# sheet1 = book1.add_sheet('CR', cell_overwrite_ok=True)
# col1 = ('行', '列')
# for i in range(0, 2):
#     sheet1.write(0, i, col1[i])
# datalist1 = zip(d1, idl_shuns_cell)
# list_new1 = list(list(arr) for arr in datalist1)
# for i in range(0, len(d1)):
#     data1 = list_new1[i]
#     for j in range(0, 2):
#         sheet1.write(i + 1, j, data1[j])
# savepath1 = '全局最大瞬时.xls'
# book1.save(savepath1)

# d2 = []
# for i in range(w):
#     d2.append(i + 1)
# book2 = xlwt.Workbook(encoding='utf-8', style_compression=0)
# sheet2 = book2.add_sheet('CR', cell_overwrite_ok=True)
# col2 = ('行', '列')
# for i in range(0, 2):
#     sheet2.write(0, i, col1[i])
# datalist2 = zip(d2, glob_cell_idl)
# list_new2 = list(list(arr) for arr in datalist2)
# for i in range(0, len(d2)):
#     data2 = list_new2[i]
#     for j in range(0, 2):
#         sheet2.write(i + 1, j, data2[j])
# savepath2 = '全局平均.xls'
# book2.save(savepath2)
