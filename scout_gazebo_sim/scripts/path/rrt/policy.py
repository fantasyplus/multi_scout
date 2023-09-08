import os
from skimage.transform import resize
import random
import numpy as np
import matplotlib.pyplot as plt


def ava_filter(x, filt_length):
    N = len(x)
    res = []
    for i in range(N):
        if i <= filt_length // 2 or i >= N - (filt_length // 2):
            temp = x[i]
        else:
            sum = 0
            for j in range(filt_length):
                sum += x[i - filt_length // 2 + j]
            temp = sum * 1.0 / filt_length
        res.append(temp)
    return res


txt = np.loadtxt("../../path.txt", delimiter=' ', encoding='utf-8')
txt = np.loadtxt("2.txt", delimiter=' ', encoding='utf-8')
x,y = txt[:,0],txt[:,1]
dx = []
dy = []
for i in range(len(x)-1):
    dx.append(x[i+1]-x[i])
    dy.append(y[i+1]-y[i])

l = 0
for i in range(len(x)-1):
    l += np.linalg.norm((dx[i],dy[i]))
print(l)
plt.plot(x,y)
plt.axis('equal')
plt.grid()
plt.show()
# k_mean = 40
# txt = np.loadtxt("epReward.txt", delimiter=' ', encoding='utf-8')
# stepList,critic_loss_List = txt[:,0],txt[:,1]
# actor_loss_List = ava_filter(critic_loss_List, k_mean)
# # critic_loss_List[0:1500] = 0
# # actor_loss_List[0:1500] = 0
# fig,ax1 = plt.subplots(dpi=400)     
# color = 'tab:blue'
# ax1.plot(stepList,critic_loss_List,color=color,linewidth=0.05) 
# # ax1.plot(stepList,ava_filter(critic_loss_List, k_mean),color=color,linewidth=0.5) 

# ax1.set_xlabel("EPISODE")
# ax1.set_ylabel("Reward-episode",color=color)  
# ax1.tick_params(axis='y',labelcolor=color)

# ax2 = ax1.twinx()
# color = 'tab:green'
# ax2.plot(stepList,actor_loss_List,color=color,linewidth=0.5) 
# # ax2.plot(stepList,ava_filter(actor_loss_List, k_mean),color=color,linewidth=0.5) 
# ax2.set_ylabel("Reward-average",color=color)  
# ax2.tick_params(axis='y',labelcolor=color)

# fig.tight_layout()
# plt.savefig('Reward.png')
# # plt.show()

# k_mean = 250
# txt = np.loadtxt("Loss.txt", delimiter=' ', encoding='utf-8')
# stepList,critic_loss_List = txt[:,0],txt[:,1]
# actor_loss_List = ava_filter(critic_loss_List, k_mean)
# # critic_loss_List[0:1500] = 0
# # actor_loss_List[0:1500] = 0
# fig,ax1 = plt.subplots(dpi=400)     
# color = 'tab:blue'
# # ax1.plot(stepList[k_mean:-k_mean],-critic_loss_List[k_mean:-k_mean],color=color,linewidth=0.5) 
# ax1.plot(stepList[k_mean:-k_mean],critic_loss_List[-k_mean:-len(critic_loss_List)+k_mean:-1],color=color,linewidth=0.5) 
# # ax1.plot(stepList,ava_filter(critic_loss_List, k_mean),color=color,linewidth=0.5) 

# ax1.set_xlabel("EPISODE")
# ax1.set_ylabel("Loss-episode",color=color)  
# # ax1.set_ylim(-1,1)  
# ax1.set_xlim(0,47500)  
# ax1.tick_params(axis='y',labelcolor=color)

# # ax2 = ax1.twinx()
# # color = 'tab:green'
# # ax2.plot(stepList[k_mean:-k_mean],actor_loss_List[k_mean:-k_mean],color=color,linewidth=0.5) 
# # # ax2.plot(stepList,ava_filter(actor_loss_List, k_mean),color=color,linewidth=0.5) 
# # ax2.set_ylabel("Loss-average",color=color)  
# # ax2.set_ylim(0,0.5)  
# # ax2.tick_params(axis='y',labelcolor=color)
# fig.tight_layout()

# from mpl_toolkits.axes_grid1.inset_locator import inset_axes,mark_inset
# from matplotlib.patches import ConnectionPatch
# axins = inset_axes(ax1,width="60%",height="30%",loc='center left',\
#     bbox_to_anchor=(0.3,0.1,1,1),bbox_transform=ax1.transAxes)
# axins.plot(stepList[20000:-k_mean],critic_loss_List[-20000:-len(critic_loss_List)+k_mean:-1],color='orange',linewidth=0.5)
# axins.set_xlim(30000,len(critic_loss_List)-k_mean)
# axins.set_ylim(-2,48)
# mark_inset(ax1,axins,loc1=3,loc2=4,fc="none",ec='r',lw=0.5)
# plt.savefig('Loss.png')
# # plt.show()
