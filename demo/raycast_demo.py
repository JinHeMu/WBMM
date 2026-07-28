import matplotlib.pyplot as plt
import numpy as np
import time


class RayCaster2D:

    def __init__(self, start, end):

        self.start = np.array(start, dtype=float)
        self.end = np.array(end, dtype=float)

        self.direction = self.end - self.start


    def traverse(self):

        """
        Amanatides & Woo 2D版本
        """

        x0, y0 = self.start
        dx, dy = self.direction


        # 当前grid坐标

        x = int(np.floor(x0))
        y = int(np.floor(y0))


        end_x = int(np.floor(self.end[0]))
        end_y = int(np.floor(self.end[1]))


        voxels=[]


        # x方向步长

        if dx > 0:
            step_x = 1
        else:
            step_x = -1


        if dy > 0:
            step_y = 1
        else:
            step_y = -1



        # 到x边界距离

        if dx !=0:

            if dx>0:
                next_x = x+1
            else:
                next_x = x

            tMaxX=(next_x-x0)/dx

            tDeltaX=abs(1/dx)

        else:

            tMaxX=np.inf
            tDeltaX=np.inf



        # 到y边界距离

        if dy!=0:

            if dy>0:
                next_y=y+1
            else:
                next_y=y

            tMaxY=(next_y-y0)/dy

            tDeltaY=abs(1/dy)

        else:

            tMaxY=np.inf
            tDeltaY=np.inf



        while True:


            voxels.append((x,y))


            if x==end_x and y==end_y:
                break



            # 判断穿过哪个边界

            if tMaxX < tMaxY:

                x += step_x

                tMaxX += tDeltaX


            else:

                y += step_y

                tMaxY += tDeltaY



        return voxels




# ===============================
# 地图
# ===============================


size=10


# 传感器

start=(1.5,1.5)


# 障碍物

end=(8.2,6.5)



caster=RayCaster2D(start,end)


voxels=caster.traverse()



print("Ray经过的voxel:")
print(voxels)



# ===============================
# 可视化
# ===============================


fig,ax=plt.subplots(figsize=(7,7))


# 画grid

for i in range(size+1):

    ax.plot(
        [0,size],
        [i,i],
        color='gray'
    )

    ax.plot(
        [i,i],
        [0,size],
        color='gray'
    )


# obstacle

ax.scatter(
    end[0],
    end[1],
    s=200,
    marker='X',
    label="Obstacle"
)



# sensor

ax.scatter(
    start[0],
    start[1],
    s=150,
    label="Camera"
)



# 动态显示ray


for idx,(x,y) in enumerate(voxels):


    rect=plt.Rectangle(
        (x,y),
        1,
        1,
        fill=True,
        alpha=0.4
    )

    ax.add_patch(rect)


    ax.text(
        x+0.5,
        y+0.5,
        str(idx),
        ha='center',
        va='center'
    )


    ax.plot(
        [start[0],end[0]],
        [start[1],end[1]]
    )


    plt.pause(0.5)



ax.set_xlim(0,size)
ax.set_ylim(0,size)

ax.set_aspect("equal")

ax.legend()

plt.show()