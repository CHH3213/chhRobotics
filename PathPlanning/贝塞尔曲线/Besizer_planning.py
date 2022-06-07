import numpy as np
import matplotlib.pyplot as plt
import copy
from celluloid import Camera  # 保存动图时用，pip install celluloid


from n_Bezier import bezier # 导出贝塞尔曲线生成函数

if __name__=='__main__':
    d = 3.5  # 道路标准宽度

    # 控制点
    Ps = np.array([
        [0, -d / 2],
        [25, -d / 2],
        [25, d / 2],
        [50, d / 2]
        ])

    n = len(Ps) - 1  # 贝塞尔曲线的阶数

    path=[]  # 路径点存储

    for t in np.arange(0,1.01,0.01):
        p_t = bezier(Ps,len(Ps),t)
        path.append(p_t)
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

        plt.plot(Ps[:, 0], Ps[:, 1], 'ro') # 画控制点
        plt.plot(Ps[:, 0], Ps[:, 1], 'y') # 画控制点连线
        # 设置坐标轴显示范围
        # plt.axis('equal')
        plt.gca().set_aspect('equal')
        # 绘制路径

        plt.plot(path[0:i, 0], path[0:i, 1], 'g')  # 路径点
        plt.pause(0.001)
    #     camera.snap()  # 录制动图
    # animation = camera.animate()
    # animation.save('trajectory.gif')
