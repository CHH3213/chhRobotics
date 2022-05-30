import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import math
from celluloid import Camera  # 保存动图时用，pip install celluloid

k=0.2 # 增益系数
dt=0.1 # 时间间隔，单位：s

L=2 # 车辆轴距，单位：m
v = 2 # 初始速度
x_0=0 # 初始x
y_0=-3 #初始y
psi_0=0 # 初始航向角

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

def cal_target_index(robot_state, refer_path):
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


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    copied from https://atsushisakai.github.io/PythonRobotics/modules/path_tracking/stanley_control/stanley_control.html
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def stanley_control(robot_state,refer_path, refer_path_psi):
    """stanley控制

    Args:
        robot_state (_type_): 机器人位姿，包括x,y,yaw,v
        refer_path (_type_): 参考轨迹的位置
        refer_path_psi (_type_): 参考轨迹上点的切线方向的角度
        last_target_index (_type_): 上一个目标临近点

    Returns:
        _type_: _description_
    """
    current_target_index = cal_target_index(robot_state[0:2],refer_path)

    
    # 当计算出来的目标临近点索引大于等于参考轨迹上的最后一个点索引时
    if current_target_index>=len(refer_path):  
        current_target_index=len(refer_path)-1 
        current_ref_point = refer_path[-1] 
        psi_t = refer_path_psi[-1]
    else:
        # print(current_target_index)
        current_ref_point=refer_path[current_target_index]
        psi_t = refer_path_psi[current_target_index]
    
    # 计算横向误差e_y
    # 1. 参考自https://blog.csdn.net/renyushuai900/article/details/98460758
    # if(robot_state[0]-current_ref_point[0])*psi_t-(robot_state[1]-current_ref_point[1])>0:
    # 2. 
    if(robot_state[1]-current_ref_point[1])*math.cos(psi_t)-(robot_state[0]-current_ref_point[0])*math.sin(psi_t)<=0:

        e_y=np.linalg.norm(robot_state[0:2]-current_ref_point)
    else:
        e_y = -np.linalg.norm(robot_state[0:2]-current_ref_point)


    # 通过公式(5)计算转角,符号保持一致
    psi = robot_state[2]
    v = robot_state[3]
    # psi_t的计算我看还有直接这么计算的
    # psi_t = math.atan2(current_ref_point[1]-robot_state[1],current_ref_point[0]-robot_state[0])
    theta_e = psi_t-psi
    delta_e = math.atan2(k*e_y,v)
    delta = normalize_angle(theta_e+delta_e)
    return delta,current_target_index

    





def main():
    # set reference trajectory
    refer_path = np.zeros((1000, 2))
    refer_path[:, 0] = np.linspace(0, 100, 1000)  # 直线
    refer_path[:, 1] = 2*np.sin(refer_path[:, 0]/3.0) +  2.5*np.cos(refer_path[:, 0]/2.0)  # 生成正弦轨迹
    refer_path_psi = [math.atan2(refer_path[i+1,1]-refer_path[i,1],refer_path[i+1,0]-refer_path[i,0]) for i in range(len(refer_path)-1)] # 参考轨迹上点的切线方向的角度,近似计算

    # 运动学模型
    ugv = KinematicModel_3(x_0, y_0, psi_0, v, L, dt)
    goal = refer_path[-2]

    x_ = []
    y_ = []
    fig = plt.figure(1)
    # 保存动图用
    camera = Camera(fig)
    # plt.ylim([-3,3])
    for _ in range(500):
        robot_state = np.zeros(4)
        robot_state[0] = ugv.x
        robot_state[1] = ugv.y
        robot_state[2]=ugv.psi
        robot_state[3]=ugv.v


        delta,ind = stanley_control(robot_state,refer_path,refer_path_psi)

        ugv.update_state(0, delta)  # 加速度设为0，恒速

        x_.append(ugv.x)
        y_.append(ugv.y)

        # 显示动图
        plt.cla()
        plt.plot(refer_path[:, 0], refer_path[:, 1], '-.b', linewidth=1.0)
        plt.plot(x_, y_, "-r", label="trajectory")
        plt.plot(refer_path[ind,0], refer_path[ind,1], "go", label="target")
        # plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)
    #     camera.snap()
        if ind>=len(refer_path_psi)-1:
            break
    # animation = camera.animate()
    # animation.save('trajectory.gif')
    plt.figure(2)
    plt.plot(refer_path[:, 0], refer_path[:, 1], '-.b', linewidth=1.0)
    plt.plot(x_, y_, 'r')
    plt.show()


if __name__ == '__main__':
    main()
