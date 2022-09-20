from celluloid import Camera # 保存动图时用，pip install celluloid
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import math

N=100 # 迭代范围
EPS = 1e-4 # 迭代精度
Q = np.eye(3)*3
R = np.eye(2)*2.
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

  def state_space(self, ref_delta, ref_yaw):
    """将模型离散化后的状态空间表达

    Args:
        delta (_type_): 参考输入

    Returns:
        _type_: _description_
    """
    A = np.matrix([
        [1.0, 0.0, -self.v*self.dt*math.sin(ref_yaw)],
        [0.0, 1.0, self.v*self.dt*math.cos(ref_yaw)],
        [0.0, 0.0, 1.0]])

    B = np.matrix([
        [self.dt*math.cos(ref_yaw), 0],
        [self.dt*math.sin(ref_yaw), 0],
        [self.dt*math.tan(ref_delta)/self.L, self.v*self.dt /
         (self.L*math.cos(ref_delta)*math.cos(ref_delta))]
    ])
    return A, B


class MyReferencePath:
    def __init__(self):
        # set reference trajectory
        # refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k 
        self.refer_path = np.zeros((1000, 4))
        self.refer_path[:,0] = np.linspace(0, 100, 1000) # x
        self.refer_path[:,1] = 2*np.sin(self.refer_path[:,0]/3.0)+2.5*np.cos(self.refer_path[:,0]/2.0) # y
        # 使用差分的方式计算路径点的一阶导和二阶导，从而得到切线方向和曲率
        for i in range(len(self.refer_path)):
            if i == 0:
                dx = self.refer_path[i+1,0] - self.refer_path[i,0]
                dy = self.refer_path[i+1,1] - self.refer_path[i,1]
                ddx = self.refer_path[2,0] + self.refer_path[0,0] - 2*self.refer_path[1,0]
                ddy = self.refer_path[2,1] + self.refer_path[0,1] - 2*self.refer_path[1,1]
            elif i == (len(self.refer_path)-1):
                dx = self.refer_path[i,0] - self.refer_path[i-1,0]
                dy = self.refer_path[i,1] - self.refer_path[i-1,1]
                ddx = self.refer_path[i,0] + self.refer_path[i-2,0] - 2*self.refer_path[i-1,0]
                ddy = self.refer_path[i,1] + self.refer_path[i-2,1] - 2*self.refer_path[i-1,1]
            else:      
                dx = self.refer_path[i+1,0] - self.refer_path[i,0]
                dy = self.refer_path[i+1,1] - self.refer_path[i,1]
                ddx = self.refer_path[i+1,0] + self.refer_path[i-1,0] - 2*self.refer_path[i,0]
                ddy = self.refer_path[i+1,1] + self.refer_path[i-1,1] - 2*self.refer_path[i,1]
            self.refer_path[i,2]=math.atan2(dy,dx) # yaw
            # 计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
            # 参考：https://blog.csdn.net/weixin_46627433/article/details/123403726
            self.refer_path[i,3]=(ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2)) # 曲率k计算
            
    def calc_track_error(self, x, y):
        """计算跟踪误差

        Args:
            x (_type_): 当前车辆的位置x
            y (_type_): 当前车辆的位置y

        Returns:
            _type_: _description_
        """
        # 寻找参考轨迹最近目标点
        d_x = [self.refer_path[i,0]-x for i in range(len(self.refer_path))] 
        d_y = [self.refer_path[i,1]-y for i in range(len(self.refer_path))] 
        d = [np.sqrt(d_x[i]**2+d_y[i]**2) for i in range(len(d_x))]
        s = np.argmin(d) # 最近目标点索引


        yaw = self.refer_path[s, 2]
        k = self.refer_path[s, 3]
        angle = normalize_angle(yaw - math.atan2(d_y[s], d_x[s]))
        e = d[s]  # 误差
        if angle < 0:
            e *= -1

        return e, k, yaw, s
        
        
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


def cal_Ricatti(A,B,Q,R):
    """解代数里卡提方程

    Args:
        A (_type_): 状态矩阵A
        B (_type_): 状态矩阵B
        Q (_type_): Q为半正定的状态加权矩阵, 通常取为对角阵；Q矩阵元素变大意味着希望跟踪偏差能够快速趋近于零；
        R (_type_): R为正定的控制加权矩阵，R矩阵元素变大意味着希望控制输入能够尽可能小。

    Returns:
        _type_: _description_
    """
    # 设置迭代初始值
    Qf=Q
    P=Qf
    # 循环迭代
    for t in range(N):
        P_=Q+A.T@P@A-A.T@P@B@np.linalg.pinv(R+B.T@P@B)@B.T@P@A
        if(abs(P_-P).max()<EPS):
            break
        P=P_
    return P_


def lqr(robot_state, refer_path, s0, A, B, Q, R):
    """
    LQR控制器
    """
    # x为位置和航向误差
    x=robot_state[0:3]-refer_path[s0,0:3]


    P = cal_Ricatti(A,B,Q,R)



    K = -np.linalg.pinv(R + B.T @ P @ B) @ B.T @ P @ A
    u = K @ x
    u_star = u #u_star = [[v-ref_v,delta-ref_delta]] 
    # print(u_star)
    return u_star[0,1]



# 使用随便生成的轨迹


def main():

    reference_path = MyReferencePath()
    goal = reference_path.refer_path[-1, 0:2]

    # 运动学模型
    ugv = KinematicModel_3(x_0, y_0, psi_0, v, L, dt)
    x_ = []
    y_ = []
    fig = plt.figure(1)
    # 保存动图用
    camera = Camera(fig)
    # plt.ylim([-3,3])
    for i in range(500):
        robot_state = np.zeros(4)
        robot_state[0] = ugv.x
        robot_state[1] = ugv.y
        robot_state[2] = ugv.psi
        robot_state[3] = ugv.v
        e, k, ref_yaw, s0 = reference_path.calc_track_error(
            robot_state[0], robot_state[1])
        ref_delta = math.atan2(L*k, 1)
        A, B = ugv.state_space(ref_delta, ref_yaw)
        delta = lqr(robot_state, reference_path.refer_path, s0, A, B, Q, R)
        delta = delta+ref_delta

        ugv.update_state(0, delta)  # 加速度设为0，恒速

        x_.append(ugv.x)
        y_.append(ugv.y)

        # 显示动图
        plt.cla()
        plt.plot(reference_path.refer_path[:, 0], reference_path.refer_path[:,
                 1], "-.b",  linewidth=1.0, label="course")
        plt.plot(x_, y_, "-r", label="trajectory")
        plt.plot(reference_path.refer_path[s0, 0],
                 reference_path.refer_path[s0, 1], "go", label="target")
        # plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)

        # camera.snap()
        # 判断是否到达最后一个点
        if np.linalg.norm(robot_state[0:2]-goal) <= 0.1:
            print("reach goal")
            break
    # animation = camera.animate()
    # animation.save('trajectory.gif')


if __name__ == '__main__':
    main()
