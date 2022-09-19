from celluloid import Camera # 保存动图时用，pip install celluloid
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import cvxpy
import math


# mpc parameters
NX = 3  # x = x, y, yaw
NU = 2  # u = [v,delta]
T = 8  # horizon length
R = np.diag([0.1, 0.1])  # input cost matrix
Rd = np.diag([0.1, 0.1])  # input difference cost matrix
Q = np.diag([1, 1, 1])  # state cost matrix
Qf = Q  # state final matrix


#车辆
dt = 0.1  # 时间间隔，单位：s
L = 2  # 车辆轴距，单位：m
v = 2  # 初始速度
x_0 = 0  # 初始x
y_0 = -3  # 初始y
psi_0 = 0  # 初始航向角

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(45.0)  # maximum steering speed [rad/s]

MAX_VEL = 2.0  # maximum accel [m/s]

def get_nparray_from_matrix(x):
    return np.array(x).flatten()


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
        ref_delta (_type_): 参考的转角控制量
        ref_yaw (_type_): 参考的偏航角

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
        [self.dt*math.tan(ref_delta)/self.L, self.v*self.dt /(self.L*math.cos(ref_delta)*math.cos(ref_delta))]
    ])

    C = np.eye(3)
    return A, B, C


class MyReferencePath:
    def __init__(self):
        # set reference trajectory
        # refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k
        self.refer_path = np.zeros((1000, 4))
        self.refer_path[:, 0] = np.linspace(0, 100, 1000)  # x
        self.refer_path[:, 1] = 2*np.sin(self.refer_path[:, 0]/3.0) + \
            2.5*np.cos(self.refer_path[:, 0]/2.0)  # y
        # 使用差分的方式计算路径点的一阶导和二阶导，从而得到切线方向和曲率
        for i in range(len(self.refer_path)):
            if i == 0:
                dx = self.refer_path[i+1, 0] - self.refer_path[i, 0]
                dy = self.refer_path[i+1, 1] - self.refer_path[i, 1]
                ddx = self.refer_path[2, 0] + \
                    self.refer_path[0, 0] - 2*self.refer_path[1, 0]
                ddy = self.refer_path[2, 1] + \
                    self.refer_path[0, 1] - 2*self.refer_path[1, 1]
            elif i == (len(self.refer_path)-1):
                dx = self.refer_path[i, 0] - self.refer_path[i-1, 0]
                dy = self.refer_path[i, 1] - self.refer_path[i-1, 1]
                ddx = self.refer_path[i, 0] + \
                    self.refer_path[i-2, 0] - 2*self.refer_path[i-1, 0]
                ddy = self.refer_path[i, 1] + \
                    self.refer_path[i-2, 1] - 2*self.refer_path[i-1, 1]
            else:
                dx = self.refer_path[i+1, 0] - self.refer_path[i, 0]
                dy = self.refer_path[i+1, 1] - self.refer_path[i, 1]
                ddx = self.refer_path[i+1, 0] + \
                    self.refer_path[i-1, 0] - 2*self.refer_path[i, 0]
                ddy = self.refer_path[i+1, 1] + \
                    self.refer_path[i-1, 1] - 2*self.refer_path[i, 1]
            self.refer_path[i, 2] = math.atan2(dy, dx)  # yaw
            # 计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
            # 参考：https://blog.csdn.net/weixin_46627433/article/details/123403726
            self.refer_path[i, 3] = (
                ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))  # 曲率k计算

    def calc_track_error(self, x, y):
        """计算跟踪误差

        Args:
            x (_type_): 当前车辆的位置x
            y (_type_): 当前车辆的位置y

        Returns:
            _type_: _description_
        """
        # 寻找参考轨迹最近目标点
        d_x = [self.refer_path[i, 0]-x for i in range(len(self.refer_path))]
        d_y = [self.refer_path[i, 1]-y for i in range(len(self.refer_path))]
        d = [np.sqrt(d_x[i]**2+d_y[i]**2) for i in range(len(d_x))]
        s = np.argmin(d)  # 最近目标点索引

        yaw = self.refer_path[s, 2]
        k = self.refer_path[s, 3]
        angle = normalize_angle(yaw - math.atan2(d_y[s], d_x[s]))
        e = d[s]  # 误差
        if angle < 0:
            e *= -1

        return e, k, yaw, s

    def calc_ref_trajectory(self, robot_state, dl=1.0):
        """计算参考轨迹点，统一化变量数组，便于后面MPC优化使用
            参考自https://github.com/AtsushiSakai/PythonRobotics/blob/eb6d1cbe6fc90c7be9210bf153b3a04f177cc138/PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py
        Args:
            robot_state (_type_): 车辆的状态(x,y,yaw,v)
            dl (float, optional): _description_. Defaults to 1.0.

        Returns:
            _type_: _description_
        """
        e, k, ref_yaw, ind = self.calc_track_error(
            robot_state[0], robot_state[1])

        xref = np.zeros((NX, T + 1))
        dref = np.zeros((NU, T))
        ncourse = len(self.refer_path)

        xref[0, 0] = self.refer_path[ind, 0]
        xref[1, 0] = self.refer_path[ind, 1]
        xref[2, 0] = self.refer_path[ind, 2]

        # 参考控制量[v,delta]
        ref_delta = math.atan2(L*k, 1)
        dref[0, :] = robot_state[3]
        dref[1, :] = ref_delta

        travel = 0.0

        for i in range(T + 1):
            travel += abs(robot_state[3]) * dt
            dind = int(round(travel / dl))

            if (ind + dind) < ncourse:
                xref[0, i] = self.refer_path[ind + dind, 0]
                xref[1, i] = self.refer_path[ind + dind, 1]
                xref[2, i] = self.refer_path[ind + dind, 2]

            else:
                xref[0, i] = self.refer_path[ncourse - 1, 0]
                xref[1, i] = self.refer_path[ncourse - 1, 1]
                xref[2, i] = self.refer_path[ncourse - 1, 2]

        return xref, ind, dref


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


def linear_mpc_control(xref, x0, delta_ref, ugv):
    """
    linear mpc control

    xref: reference point
    x0: initial state
    delta_ref: reference steer angle
    ugv:车辆对象
    returns: 最优的控制量和最优状态
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0  # 代价函数
    constraints = []  # 约束条件

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t]-delta_ref[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(x[:, t] - xref[:, t], Q)

        A, B, C = ugv.state_space(delta_ref[1, t], xref[2, t])
        constraints += [x[:, t + 1]-xref[:, t+1] == A @
                        (x[:, t]-xref[:, t]) + B @ (u[:, t]-delta_ref[:, t])]


    cost += cvxpy.quad_form(x[:, T] - xref[:, T], Qf)

    constraints += [(x[:, 0]) == x0]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_VEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        opt_x = get_nparray_from_matrix(x.value[0, :])
        opt_y = get_nparray_from_matrix(x.value[1, :])
        opt_yaw = get_nparray_from_matrix(x.value[2, :])
        opt_v = get_nparray_from_matrix(u.value[0, :])
        opt_delta = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        opt_v, opt_delta, opt_x, opt_y, opt_yaw = None, None, None, None, None,

    return opt_v, opt_delta, opt_x, opt_y, opt_yaw



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
        x0 = robot_state[0:3]
        xref, target_ind, dref = reference_path.calc_ref_trajectory(
            robot_state)
        opt_v, opt_delta, opt_x, opt_y, opt_yaw = linear_mpc_control(
            xref, x0, dref, ugv)
        ugv.update_state(0, opt_delta[0])  # 加速度设为0，恒速

        x_.append(ugv.x)
        y_.append(ugv.y)

        # 显示动图
        plt.cla()
        plt.plot(reference_path.refer_path[:, 0], reference_path.refer_path[:,
                 1], "-.b",  linewidth=1.0, label="course")
        plt.plot(x_, y_, "-r", label="trajectory")
        plt.plot(reference_path.refer_path[target_ind, 0],
                 reference_path.refer_path[target_ind, 1], "go", label="target")
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


if __name__=='__main__':
    main()