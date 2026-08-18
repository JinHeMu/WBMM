import numpy as np
import matplotlib.pyplot as plt

from scipy.optimize import minimize


# =============================
# 环境
# =============================

start = np.array([0,0])
goal  = np.array([10,10])


# 障碍物
obstacles = [
    np.array([5,5]),
    np.array([7,6])
]

obs_radius = 1.2


# =============================
# 初始轨迹
# =============================

N = 40


x = np.linspace(
    start[0],
    goal[0],
    N
)

y = np.linspace(
    start[1],
    goal[1],
    N
)


# 加一点扰动模拟粗路径

y += np.sin(np.linspace(0,3*np.pi,N))*0.8


traj0 = np.vstack(
    [x,y]
).T



# =============================
# 代价函数
# =============================

def cost_function(z):

    traj = z.reshape(N,2)


    cost = 0


    # -------------------------
    # 1. 平滑约束
    # -------------------------

    for i in range(1,N-1):

        smooth = (
            traj[i+1]
            -2*traj[i]
            +traj[i-1]
        )

        cost += 10*np.sum(
            smooth**2
        )


    # -------------------------
    # 2. 避障约束
    # -------------------------

    for p in traj:

        for obs in obstacles:

            d=np.linalg.norm(
                p-obs
            )

            violation = (
                obs_radius-d
            )


            if violation>0:

                cost += 2000*violation**2



    # -------------------------
    # 3. 终点约束
    # -------------------------

    cost += 100*np.sum(
        (traj[-1]-goal)**2
    )


    # -------------------------
    # 4. 起点约束
    # -------------------------

    cost += 100*np.sum(
        (traj[0]-start)**2
    )


    # -------------------------
    # 5. 速度约束
    # -------------------------

    vmax=0.5


    for i in range(N-1):

        v=np.linalg.norm(
            traj[i+1]-traj[i]
        )


        if v>vmax:

            cost+=20*(v-vmax)**2


    return cost



# =============================
# L-BFGS优化
# =============================


result = minimize(
    cost_function,
    traj0.reshape(-1),
    method="L-BFGS-B",
    options={
        "maxiter":500
    }
)



traj_opt=result.x.reshape(N,2)


print(
    "优化前代价:",
    cost_function(traj0.reshape(-1))
)

print(
    "优化后代价:",
    cost_function(traj_opt.reshape(-1))
)



# =============================
# 可视化
# =============================

plt.figure(figsize=(8,8))


# 障碍

for obs in obstacles:

    circle=plt.Circle(
        obs,
        obs_radius,
        color="gray",
        alpha=0.5
    )

    plt.gca().add_patch(circle)



# 原始轨迹

plt.plot(
    traj0[:,0],
    traj0[:,1],
    "r--",
    label="initial trajectory"
)



# 优化轨迹

plt.plot(
    traj_opt[:,0],
    traj_opt[:,1],
    "b",
    linewidth=3,
    label="optimized trajectory"
)



plt.scatter(
    start[0],
    start[1],
    c="green",
    s=100,
    label="start"
)


plt.scatter(
    goal[0],
    goal[1],
    c="black",
    s=100,
    label="goal"
)


plt.axis("equal")
plt.grid()
plt.legend()

plt.title(
    "L-BFGS Trajectory Optimization"
)

plt.show()