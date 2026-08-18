import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
import time


# =====================
# 地图参数
# =====================

size = 50


# 0 free
# 1 obstacle

occ = np.zeros((size,size))


# 添加障碍物墙

occ[25:40,30]=1
occ[25:40,31]=1
occ[25:40,32]=1



# 机器人位置

robot=np.array([10,10])



# =====================
# 简化Raycast
# =====================

def raycast(start,end):

    points=[]

    x0,y0=start
    x1,y1=end


    dx=x1-x0
    dy=y1-y0


    steps=max(abs(dx),abs(dy))


    for i in range(int(steps)):

        t=i/steps

        x=int(x0+t*dx)
        y=int(y0+t*dy)

        points.append((x,y))


    return points



# =====================
# ESDF计算
# =====================

def compute_esdf(occ):

    # obstacle为True

    obstacle = occ>0


    # 到障碍距离

    dist = distance_transform_edt(
        ~obstacle
    )


    return dist



# =====================
# 动态显示
# =====================


plt.ion()

fig,ax=plt.subplots(
    figsize=(7,7)
)



targets=[
    (32,20),
    (32,25),
    (32,30),
    (32,35)
]


for target in targets:


    # raycast

    ray=raycast(
        robot,
        target
    )


    # 更新free空间

    for x,y in ray[:-1]:

        occ[y,x]=0


    # 终点障碍

    x,y=target

    occ[y,x]=1



    # ESDF

    esdf=compute_esdf(occ)



    ax.clear()


    # 显示ESDF

    ax.imshow(
        esdf,
        cmap="viridis",
        origin="lower"
    )


    # 障碍

    oy,ox=np.where(occ>0)

    ax.scatter(
        ox,
        oy,
        marker="s",
        s=30
    )


    # robot

    ax.scatter(
        robot[0],
        robot[1],
        s=100,
        marker="o"
    )


    # ray

    rx=[p[0] for p in ray]
    ry=[p[1] for p in ray]


    ax.plot(
        rx,
        ry
    )


    ax.set_title(
        "Occupancy + ESDF update"
    )


    plt.pause(1)



plt.ioff()

plt.show()