from scipy.spatial import KDTree
from celluloid import Camera  # 保存动图时用，pip install celluloid
from PID_controller import PID_posi_2, PID_inc
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import math
class KinematicModel_3:
  """假设控制量为转向角delta_f和加速度a
  """

  def __init__(self, x, y, psi, v, L, dt):
    self.x = x
    self.y = y
    self.psi = psi
    self.v = v
    self.L = L
    # 实现是离散的模型
    self.dt = dt

  def update_state(self, a, delta_f):
    self.x = self.x+self.v*math.cos(self.psi)*self.dt
    self.y = self.y+self.v*math.sin(self.psi)*self.dt
    self.psi = self.psi+self.v/self.L*math.tan(delta_f)*self.dt
    self.v = self.v+a*self.dt

  def get_state(self):
    return self.x, self.y, self.psi, self.v


## 位置式
PID = PID_posi_2(k=[2, 0.01, 30], target=0, upper=np.pi/6, lower=-np.pi/6)
## 增量式
# PID = PID_inc(k=[2.5, 0.175, 30], target=0, upper=np.pi/6, lower=-np.pi/6)

def cal_target_index(robot_state,refer_path):
    """得到临近的路点

    Args:
        robot_state (_type_): 当前车辆位置
        refer_path (_type_): 参考轨迹（数组）

    Returns:
        _type_: 最近的路点的索引
    """
    dists = []
    for xy in refer_path:
        dis = np.linalg.norm(robot_state-xy)
        dists.append(dis)

    min_index = np.argmin(dists)
    return min_index

def main():
    # set reference trajectory
    refer_path = np.zeros((1000, 2))
    refer_path[:, 0] = np.linspace(0, 100, 1000)  # 直线
    # +2.5*np.cos(refer_path[:,0]/2.0) # 生成正弦轨迹
    refer_path[:, 1] = 2*np.sin(refer_path[:, 0]/3.0)
    refer_tree = KDTree(refer_path)  # reference trajectory


    # 假设初始状态为x=0,y=-1,偏航角=0.5rad，前后轴距离2m，速度为2m/s，时间步为0.1秒
    ugv = KinematicModel_3(0, -1, 0.5, 2, 2, 0.1)
    k = 0.1
    c = 2
    x_ = []
    y_ = []
    fig = plt.figure(1)
    # 保存动图用
    camera = Camera(fig)

    for i in range(550):
        robot_state = np.zeros(2)
        robot_state[0] = ugv.x
        robot_state[1] = ugv.y
        distance, ind = refer_tree.query(robot_state)  # 在参考轨迹上查询离robot_state最近的点
        # ind = cal_target_index(robot_state,refer_path)  # 使用简单的一个函数实现查询离robot_state最近的点，耗时比较长

        alpha = math.atan2(
            refer_path[ind, 1]-robot_state[1], refer_path[ind, 0]-robot_state[0])
        l_d = np.linalg.norm(refer_path[ind]-robot_state)
        # l_d = k*ugv.v+c  # 前视距离
        theta_e = alpha-ugv.psi
        e_y = -l_d*math.sin(theta_e)  # 与博客中公式相比多了个负号，我目前还不是太理解，暂时先放着
        # e_y = -l_d*np.sign(math.sin(theta_e))  # 第二种误差表示
        # e_y = robot_state[1]-refer_path[ind, 1] #第三种误差表示
        # PID.set_target(0)
        # print(refer_path[i,1])
        delta_f = PID.cal_output(e_y)
        # print(e_y)
        # print(alpha)
        ugv.update_state(0, delta_f)  # 加速度设为0

        x_.append(ugv.x)
        y_.append(ugv.y)

        # 显示动图
        plt.cla()
        plt.plot(refer_path[:, 0], refer_path[:, 1], '-.b', linewidth=1.0)
        plt.plot(x_, y_, "-r", label="trajectory")
        plt.plot(refer_path[ind, 0], refer_path[ind, 1], "go", label="target")
        # plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)
    #     camera.snap()
    # animation = camera.animate()
    # animation.save('trajectory.gif')

    plt.figure(2)
    plt.plot(refer_path[:, 0], refer_path[:, 1], '-.b', linewidth=1.0)
    plt.plot(x_, y_, 'r')
    plt.show()

if __name__=='__main__':
    main()
