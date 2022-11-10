import numpy as np
import matplotlib.pyplot as plt
import copy
from celluloid import Camera  # 保存动图时用，pip install celluloid
import math

class Config:
    """
    simulation parameter class
    """
    def __init__(self):
        # robot parameter
        # 线速度边界
        self.v_max = 1.0  # [m/s]
        self.v_min = -0.5  # [m/s]
        # 角速度边界
        self.w_max = 40.0 * math.pi / 180.0  # [rad/s]
        self.w_min = -40.0 * math.pi / 180.0  # [rad/s]
        # 线加速度和角加速度最大值
        self.a_vmax = 0.2  # [m/ss]
        self.a_wmax = 40.0 * math.pi / 180.0  # [rad/ss]
        # 采样分辨率 
        self.v_sample = 0.01  # [m/s]
        self.w_sample = 0.1 * math.pi / 180.0  # [rad/s]
        # 离散时间
        self.dt = 0.1  # [s] Time tick for motion prediction
        # 轨迹推算时间长度
        self.predict_time = 3.0  # [s]
        # 轨迹评价函数系数
        self.alpha = 0.15
        self.beta = 1.0
        self.gamma = 1.0

        # Also used to check if goal is reached in both types
        self.robot_radius = 1.0  # [m] for collision check
        
        self.judge_distance = 10 # 若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值

        # 障碍物位置 [x(m) y(m), ....]
        self.ob = np.array([[-1, -1],
                    [0, 2],
                    [4.0, 2.0],
                    [5.0, 4.0],
                    [5.0, 5.0],
                    [5.0, 6.0],
                    [5.0, 9.0],
                    [8.0, 9.0],
                    [7.0, 9.0],
                    [8.0, 10.0],
                    [9.0, 11.0],
                    [12.0, 13.0],
                    [12.0, 12.0],
                    [15.0, 15.0],
                    [13.0, 13.0]
                    ])
        # 目标点位置
        self.target = np.array([10,10])


class DWA:
    def __init__(self,config) -> None:
        """初始化

        Args:
            config (_type_): 参数类
        """
        self.dt=config.dt
        self.v_min=config.v_min
        self.w_min=config.w_min
        self.v_max=config.v_max
        self.w_max=config.w_max
        self.predict_time = config.predict_time
        self.a_vmax = config.a_vmax
        self.a_wmax = config.a_wmax
        self.v_sample = config.v_sample # 线速度采样分辨率
        self.w_sample = config.w_sample # 角速度采样分辨率
        self.alpha = config.alpha
        self.beta = config.beta
        self.gamma = config.gamma
        self.radius = config.robot_radius
        self.judge_distance = config.judge_distance

    def dwa_control(self,state,goal,obstacle):
        """滚动窗口算法入口

        Args:
            state (_type_): 机器人当前状态--[x,y,yaw,v,w]
            goal (_type_): 目标点位置，[x,y]

            obstacle (_type_): 障碍物位置，dim:[num_ob,2]

        Returns:
            _type_: 控制量、轨迹（便于绘画）
        """
        control,trajectory = self.trajectory_evaluation(state,goal,obstacle)
        return control,trajectory


    def cal_dynamic_window_vel(self,v,w,state,obstacle):
        """速度采样,得到速度空间窗口

        Args:
            v (_type_): 当前时刻线速度
            w (_type_): 当前时刻角速度
            state (_type_): 当前机器人状态
            obstacle (_type_): 障碍物位置
        Returns:
            [v_low,v_high,w_low,w_high]: 最终采样后的速度空间
        """
        Vm = self.__cal_vel_limit()
        Vd = self.__cal_accel_limit(v,w)
        Va = self.__cal_obstacle_limit(state,obstacle)
        a = max([Vm[0],Vd[0],Va[0]])
        b = min([Vm[1],Vd[1],Va[1]])
        c = max([Vm[2], Vd[2],Va[2]])
        d = min([Vm[3], Vd[3],Va[3]])
        return [a,b,c,d]

    def __cal_vel_limit(self):
        """计算速度边界限制Vm

        Returns:
            _type_: 速度边界限制后的速度空间Vm
        """
        return [self.v_min,self.v_max,self.w_min,self.w_max]
    
    def __cal_accel_limit(self,v,w):
        """计算加速度限制Vd

        Args:
            v (_type_): 当前时刻线速度
            w (_type_): 当前时刻角速度
        Returns: 
            _type_:考虑加速度时的速度空间Vd
        """
        v_low = v-self.a_vmax*self.dt
        v_high = v+self.a_vmax*self.dt
        w_low = w-self.a_wmax*self.dt
        w_high = w+self.a_wmax*self.dt
        return [v_low, v_high,w_low, w_high]
    
    def __cal_obstacle_limit(self,state,obstacle):
        """环境障碍物限制Va

        Args:
            state (_type_): 当前机器人状态
            obstacle (_type_): 障碍物位置

        Returns:
            _type_: 某一时刻移动机器人不与周围障碍物发生碰撞的速度空间Va
        """
        v_low=self.v_min
        v_high = np.sqrt(2*self._dist(state,obstacle)*self.a_vmax)
        w_low =self.w_min
        w_high = np.sqrt(2*self._dist(state,obstacle)*self.a_wmax)
        return [v_low,v_high,w_low,w_high]

    def trajectory_predict(self,state_init, v,w):
        """轨迹推算

        Args:
            state_init (_type_): 当前状态---x,y,yaw,v,w
            v (_type_): 当前时刻线速度
            w (_type_): 当前时刻线速度

        Returns:
            _type_: _description_
        """
        state = np.array(state_init)
        trajectory = state
        time = 0
        # 在预测时间段内
        while time <= self.predict_time:
            x = KinematicModel(state, [v,w], self.dt) # 运动学模型
            trajectory = np.vstack((trajectory, x))
            time += self.dt

        return trajectory

    def trajectory_evaluation(self,state,goal,obstacle):
        """轨迹评价函数,评价越高，轨迹越优

        Args:
            state (_type_): 当前状态---x,y,yaw,v,w
            dynamic_window_vel (_type_): 采样的速度空间窗口---[v_low,v_high,w_low,w_high]
            goal (_type_): 目标点位置，[x,y]
            obstacle (_type_): 障碍物位置，dim:[num_ob,2]

        Returns:
            _type_: 最优控制量、最优轨迹
        """
        G_max = -float('inf') # 最优评价
        trajectory_opt = state # 最优轨迹
        control_opt = [0.,0.] # 最优控制
        dynamic_window_vel = self.cal_dynamic_window_vel(state[3], state[4],state,obstacle) # 第1步--计算速度空间
        
        # sum_heading,sum_dist,sum_vel = 0,0,0 # 统计全部采样轨迹的各个评价之和，便于评价的归一化
        # # 在本次实验中，不进行归一化也可实现该有的效果。
        # for v in np.arange(dynamic_window_vel[0],dynamic_window_vel[1],self.v_sample):
        #     for w in np.arange(dynamic_window_vel[2], dynamic_window_vel[3], self.w_sample):   
        #         trajectory = self.trajectory_predict(state, v, w)  

        #         heading_eval = self.alpha*self.__heading(trajectory,goal)
        #         dist_eval = self.beta*self.__dist(trajectory,obstacle)
        #         vel_eval = self.gamma*self.__velocity(trajectory)
        #         sum_vel+=vel_eval
        #         sum_dist+=dist_eval
        #         sum_heading +=heading_eval

        # 在速度空间中按照预先设定的分辨率采样
        sum_heading,sum_dist,sum_vel = 1,1,1 # 不进行归一化
        for v in np.arange(dynamic_window_vel[0],dynamic_window_vel[1],self.v_sample):
            for w in np.arange(dynamic_window_vel[2], dynamic_window_vel[3], self.w_sample):

                trajectory = self.trajectory_predict(state, v, w)  # 第2步--轨迹推算

                heading_eval = self.alpha*self.__heading(trajectory,goal)/sum_heading
                dist_eval = self.beta*self.__dist(trajectory,obstacle)/sum_dist
                vel_eval = self.gamma*self.__velocity(trajectory)/sum_vel
                G = heading_eval+dist_eval+vel_eval # 第3步--轨迹评价

                if G_max<=G:
                    G_max = G
                    trajectory_opt = trajectory
                    control_opt = [v,w]

        return control_opt, trajectory_opt

    def _dist(self,state,obstacle):
        """计算当前移动机器人距离障碍物最近的几何距离

        Args:
            state (_type_): 当前机器人状态
            obstacle (_type_): 障碍物位置

        Returns:
            _type_: 移动机器人距离障碍物最近的几何距离
        """
        ox = obstacle[:, 0]
        oy = obstacle[:, 1]
        dx = state[0,None] - ox[:, None]
        dy = state[1,None] - oy[:, None]
        r = np.hypot(dx, dy)
        return np.min(r)

    def __dist(self,trajectory,obstacle):
        """距离评价函数
        表示当前速度下对应模拟轨迹与障碍物之间的最近距离；
        如果没有障碍物或者最近距离大于设定的阈值，那么就将其值设为一个较大的常数值。
        Args:
            trajectory (_type_): 轨迹，dim:[n,5]
            
            obstacle (_type_): 障碍物位置，dim:[num_ob,2]

        Returns:
            _type_: _description_
        """
        ox = obstacle[:, 0]
        oy = obstacle[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)
        return np.min(r) if np.array(r <self.radius+0.2).any() else self.judge_distance

    def __heading(self,trajectory, goal):
        """方位角评价函数
        评估在当前采样速度下产生的轨迹终点位置方向与目标点连线的夹角的误差

        Args:
            trajectory (_type_): 轨迹，dim:[n,5]
            goal (_type_): 目标点位置[x,y]

        Returns:
            _type_: 方位角评价数值
        """
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = math.pi-abs(cost_angle)

        return cost

    def __velocity(self,trajectory):
        """速度评价函数， 表示当前的速度大小，可以用模拟轨迹末端位置的线速度的大小来表示

        Args:
            trajectory (_type_): 轨迹，dim:[n,5]

        Returns:
            _type_: 速度评价
        """
        return trajectory[-1,3]


def KinematicModel(state,control,dt):
  """机器人运动学模型

  Args:
      state (_type_): 状态量---x,y,yaw,v,w
      control (_type_): 控制量---v,w,线速度和角速度
      dt (_type_): 离散时间

  Returns:
      _type_: 下一步的状态
  """
  state[0] += control[0] * math.cos(state[2]) * dt
  state[1] += control[0] * math.sin(state[2]) * dt
  state[2] += control[1] * dt
  state[3] = control[0]
  state[4] = control[1]

  return state


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")



def main(config):
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = config.target

    # input [forward speed, yaw_rate]

    trajectory = np.array(x)
    ob = config.ob
    dwa = DWA(config)
    fig=plt.figure(1)
    camera = Camera(fig)
    while True:
        u, predicted_trajectory = dwa.dwa_control(x,goal, ob)

        x = KinematicModel(x, u, config.dt)  # simulate robot
        trajectory = np.vstack((trajectory, x))  # store state history
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
        plt.plot(x[0], x[1], "xr")
        plt.plot(goal[0], goal[1], "xb")
        plt.plot(ob[:, 0], ob[:, 1], "ok")
        plot_robot(x[0], x[1], x[2], config)
        plot_arrow(x[0], x[1], x[2])
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_radius:
            print("Goal!!")
            break
        # camera.snap()
        # print(x)
        # print(u)

    print("Done")
    plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
    plt.pause(0.001)
    # camera.snap()
    # animation = camera.animate()
    # animation.save('trajectory.gif')
    plt.show()

if __name__=="__main__":
    main(Config())
