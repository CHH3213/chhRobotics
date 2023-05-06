"""B样条曲线法实现车辆轨迹规划
"""
import numpy as np
import matplotlib.pyplot as plt
import copy
from celluloid import Camera  # 保存动图时用，pip install celluloid

from B_Spline import *


if __name__=='__main__':
    ## 数据定义
    k = 3  # k阶、k-1次B样条

    flag = 3  # 1,2,3分别绘制均匀B样条曲线、准均匀B样条曲线,分段B样条

    d = 3.5  # # 道路标准宽度
    # 控制点
    P = np.array([
        [0, -d / 2],
        [10, -d / 2],
        [25, -d / 2 + 0.5],
        [25, d / 2 - 0.5],
        [40, d / 2],
    ])

    n = len(P)-1  # 控制点个数-1

    ## 生成B样条曲线

    path = []  # 路径点数据存储
    Bik_u = np.zeros((n+1, 1))

    if flag == 1:  # 均匀B样条很简单
        NodeVector = np.array([np.linspace(0, 1, n + k + 1)]
                            )  # 均匀B样条节点向量，首末值定义为 0 和 1
        # for u in np.arange(0,1,0.001):
        # u的范围为[u_{k-1},u_{n+2}],这样才是open的曲线，不然你可以使用[0,1]试试。
        for u in np.arange((k-1) / (n + k + 1), (n + 2) / (n + k + 1)+0.001, 0.001):
            for i in range(n+1):
                Bik_u[i, 0] = BaseFunction(i, k, u, NodeVector)
            p_u = P.T @ Bik_u
            path.append(p_u)
    elif flag == 2:
        NodeVector = U_quasi_uniform(n, k)
        for u in np.arange(0, 1, 0.005):
            for i in range(n+1):
                Bik_u[i, 0] = BaseFunction(i, k, u, NodeVector)
            p_u = P.T @ Bik_u
            path.append(p_u)
    elif flag == 3:
        NodeVector = U_piecewise_B_Spline(n, k)
        for u in np.arange(0, 1, 0.005):
            for i in range(n+1):
                Bik_u[i, 0] = BaseFunction(i, k, u, NodeVector)
            p_u = P.T @ Bik_u
            path.append(p_u)
    path = np.array(path)


    ## 画图
    fig = plt.figure(1)
    # plt.ylim(-4, 4)
    # plt.axis([-10, 100, -15, 15])
    camera = Camera(fig)
    len_line = 50
    # 画灰色路面图
    GreyZone = np.array([[- 5, - d - 0.5], [- 5, d + 0.5],
                        [len_line, d + 0.5], [len_line, - d - 0.5]])
    for i in range(len(path)):
        # plt.cla()

        plt.fill(GreyZone[:, 0], GreyZone[:, 1], 'gray')
        # 画分界线
        plt.plot(np.array([- 5, len_line]), np.array([0, 0]), 'w--')

        plt.plot(np.array([- 5, len_line]), np.array([d, d]), 'w')

        plt.plot(np.array([- 5, len_line]), np.array([- d, - d]), 'w')

        plt.plot(P[:, 0], P[:, 1], 'ro')
        plt.plot(P[:, 0], P[:, 1], 'y')
        # 设置坐标轴显示范围
        # plt.axis('equal')
        plt.gca().set_aspect('equal')
        # 绘制路径

        plt.plot(path[0:i, 0], path[0:i, 1], 'g')  # 路径点
        plt.pause(0.001)
    #     camera.snap()
    # animation = camera.animate()
    # animation.save('trajectory.gif')
