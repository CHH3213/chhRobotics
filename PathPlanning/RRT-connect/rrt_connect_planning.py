import math
import random

import matplotlib.pyplot as plt
import numpy as np
from celluloid import Camera  # 保存动图时用，pip install celluloid
import operator
import copy


class RRT_Connect:
    """
    Class for RRT_Connect planning
    """

    class Node:
        """
        创建节点
        """

        def __init__(self, x, y):
            self.x = x  # 节点坐标
            self.y = y
            self.path_x = [] # 路径，作为画图的数据，也可以理解成保存的边集
            self.path_y = []
            self.parent = None #父节点

    class AreaBounds:
        """区域大小
        """
        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self,
            start,
            goal,
            obstacle_list,
            rand_area,
            expand_dis=3.0,
            goal_sample_rate=5,
            max_iter=1000,
            play_area=None,
            robot_radius=0.0,
            ):
        """
        Setting Parameter

        start:起点 [x,y]
        goal:目标点 [x,y]
        obstacleList:障碍物位置列表 [[x,y,size],...]
        rand_area: 采样区域 x,y ∈ [min,max]
        play_area: 约束随机树的范围 [xmin,xmax,ymin,ymax]
        robot_radius: 机器人半径
        expand_dis: 扩展的步长
        goal_sample_rate: 采样目标点的概率，百分制.default: 5，即表示5%的概率直接采样目标点

        """
        self.start = self.Node(start[0], start[1]) # 根节点
        self.end = self.Node(goal[0], goal[1]) 
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list_1 = []
        self.node_list_2 = []
        self.robot_radius = robot_radius

    def planning(self, animation=True,camara=None):
        """
        rrt path planning

        animation: flag for animation on or off

        camara: 是否保存动图
        """

        # 将起点作为根节点x_{init}​，加入到随机树的节点集合中。
        self.node_list_1 = [self.start]
        self.node_list_2 = [self.end]
        for i in range(self.max_iter):
            # 从可行区域内随机选取一个节点q_{rand}
            rnd_node = self.sample_free()  

            # 已生成的树中利用欧氏距离判断距离q_{rand}​最近的点q_{near}。
            nearest_ind_1 = self.get_nearest_node_index(self.node_list_1, rnd_node)
            nearest_node_1 = self.node_list_1[nearest_ind_1]  
            # 从q_{near}与q_{rand}的连线方向上扩展固定步长u，得到新节点 q_{new}
            new_node_1 = self.steer(nearest_node_1, rnd_node, self.expand_dis)

            
            # 第一棵树，如果在可行区域内，且q_{near}与q_{new}之间无障碍物
            if self.is_inside_play_area(new_node_1, self.play_area) and self.obstacle_free(new_node_1, self.obstacle_list, self.robot_radius):
                self.node_list_1.append(new_node_1)
                # 扩展完第一棵树的新节点$x_{𝑛𝑒𝑤}$后，以这个新的目标点$x_{𝑛𝑒𝑤}$作为第二棵树扩展的方向。
                nearest_ind_2 = self.get_nearest_node_index(self.node_list_2, new_node_1)
                nearest_node_2 = self.node_list_2[nearest_ind_2]  
                new_node_2 = self.steer(nearest_node_2, new_node_1, self.expand_dis)
                # 第二棵树
                if self.is_inside_play_area(new_node_2, self.play_area) and self.obstacle_free(new_node_2, self.obstacle_list, self.robot_radius):
                    self.node_list_2.append(new_node_2)
                    while True:
                        new_node_2_ = self.steer(new_node_2, new_node_1, self.expand_dis)
                        if self.obstacle_free(new_node_2_, self.obstacle_list, self.robot_radius):
                            self.node_list_2.append(new_node_2_)
                            new_node_2 = new_node_2_
                        else:
                            break
                        # print([new_node_2.x,new_node_2.y], [new_node_1.x,new_node_1.y])
                        # 当$𝑞′_{𝑛𝑒𝑤}=𝑞_{𝑛𝑒𝑤}$时，表示与第一棵树相连，算法结束
                        if operator.eq([new_node_2.x,new_node_2.y], [new_node_1.x,new_node_1.y]):
                            return self.generate_final_path()
            
            # 考虑两棵树的平衡性，即两棵树的节点数的多少，交换次序选择“小”的那棵树进行扩展。
            # 不过不交换的情况下好像搜索速度还更快
            if len(self.node_list_1)>len(self.node_list_2):
                list_tmp = copy.deepcopy(self.node_list_1)
                self.node_list_1 = copy.deepcopy(self.node_list_2)
                self.node_list_2 = list_tmp


            if animation and i % 5 ==0:
                self.draw_graph(rnd_node,new_node_1, camara)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):
        """连线方向扩展固定步长查找x_new

        Args:
            from_node (_type_): x_near
            to_node (_type_): x_rand
            extend_length (_type_, optional): 扩展步长u. Defaults to float("inf").

        Returns:
            _type_: _description_
        """
        # 利用反正切计算角度, 然后利用角度和步长计算新坐标
        d, theta = self.calc_distance_and_angle(from_node, to_node)

        # 如果$q_{near}$与$q_{rand}$间的距离小于步长，则直接将$q_{rand}$作为新节点$q_{new}$
        if extend_length >= d:
            new_x = to_node.x
            new_y = to_node.y
        else:
            new_x = from_node.x+math.cos(theta)*extend_length
            new_y = from_node.y+math.sin(theta)*extend_length
        new_node_1 = self.Node(new_x,new_y)
        new_node_1.path_x = [from_node.x] # 边集
        new_node_1.path_y = [from_node.y]
        new_node_1.path_x.append(new_x)
        new_node_1.path_y.append(new_y)

        new_node_1.parent = from_node

        return new_node_1



    def generate_final_path(self):
        """生成路径
        Args:
        Returns:
            _type_: _description_
        """
        path_1 = []
        node = self.node_list_1[-1]
        while node.parent is not None:
            path_1.append([node.x, node.y])
            node = node.parent
        path_1.append([node.x, node.y])

        path_2 = []
        node = self.node_list_2[-1]
        while node.parent is not None:
            path_2.append([node.x, node.y])
            node = node.parent
        path_2.append([node.x, node.y])

        path=[]
        for i in range(len(path_1)-1,-1,-1):
            path.append(path_1[i])
        for i in range(len(path_2)):
            path.append(path_2[i])
        
        return path

    def calc_dist(self, x1, y1,x2,y2):
        """计算距离
        """
        dx = x1 - x2
        dy = y1 - y2
        return math.hypot(dx, dy)

    def sample_free(self):
        # 以（100-goal_sample_rate）%的概率随机生长，(goal_sample_rate)%的概率朝向目标点生长
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None,rnd_2=None, camera=None):
        if camera==None:
            plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        # 画随机点
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        if rnd_2 is not None:
            plt.plot(rnd_2.x, rnd_2.y, "^r")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd_2.x, rnd_2.y, self.robot_radius, '-b')
        # 画已生成的树
        for node in self.node_list_1:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
        for node in self.node_list_2:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
        # 画障碍物
        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        # 如果约定了可行区域，则画出可行区域
        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                self.play_area.xmax, self.play_area.xmin,
                self.play_area.xmin],
                [self.play_area.ymin, self.play_area.ymin,
                self.play_area.ymax, self.play_area.ymax,
                self.play_area.ymin],
                "-k")

        # 画出起点和目标点
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)
        if camera!=None:
            camera.snap()
    # 静态方法无需实例化，也可以实例化后调用，静态方法内部不能调用self.的变量
    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list_1, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list_1]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def is_inside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok

    @staticmethod
    def obstacle_free(node, obstacleList, robot_radius):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size+robot_radius)**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        """计算两个节点间的距离和方位角

        Args:
            from_node (_type_): _description_
            to_node (_type_): _description_

        Returns:
            _type_: _description_
        """
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main(gx=6.0, gy=10.0):
    print("start " + __file__)
    fig = plt.figure(1)

    camera = Camera(fig) # 保存动图时使用
    camera = None # 不保存动图时，camara为None
    show_animation = True
    # ====Search Path with RRT_Connect====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
        (6, 12, 1),
    ]  # [x,y,size(radius)]
    # Set Initial parameters
    rrt = RRT_Connect(
        start=[0, 0],
        goal=[gx, gy],
        rand_area=[-2, 15],
        obstacle_list=obstacleList,
        play_area=[0, 10, 0, 14],
        robot_radius=0.8
    )
    path = rrt.planning(animation=show_animation,camara=camera)
    if path is None:
        print("Cannot find path")
    else:
        path = np.array(path)
        print(path)
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.draw_graph(camera=camera)
            plt.grid(True)
            plt.pause(0.01)  
            plt.plot(path[:,0], path[:,1], '-r')
            if camera!=None:
                camera.snap()
                animation = camera.animate()
                animation.save('trajectory.gif')
            plt.figure(2)
            plt.axis("equal")
            plt.axis([-2, 15, -2, 15])
            plt.grid(True)
            plt.plot(path[:,0], path[:,1], '-r')
            plt.show()




if __name__ == '__main__':
    main()
