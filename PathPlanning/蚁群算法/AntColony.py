# -*- coding: utf-8 -*-
# @Time    : 2019/12/25 16:23
# @Author  : HelloWorld！
# @FileName: ant.py
# @Software: PyCharm
# @Operating System: Windows 10
# @Python.version: 3.6
# https: // blog.csdn.net/weixin_45690272/article/details/103698475
# https://blog.csdn.net/weixin_39383896/article/details/101568241
# https://blog.csdn.net/golden1314521/article/details/45059719
# https://blog.csdn.net/fanxin_i/article/details/80380733
#城市数量为M，
# 蚁群规模（蚂蚁数量）m、
# 信息素重要程度因子α、
# 启发函数重要程度因子β，即启发式信息的相对重要程度、
# 信息素会发银子ρ、
# 信息素释放总量Q、
# 最大迭代次数iter_max、迭代次数初值iter=1。
import random
import copy
import sys
import math
import tkinter
import threading
from functools import reduce
(ALPHA, BETA, RHO, Q) = (1, 2, 0.5, 100)
#城市数，小蚂蚁个数
(city_num, ant_num) = (50, 50)
#城市坐标
distance_x = [
    178, 272, 176, 171, 650, 499, 267, 703, 408, 437, 491, 74, 532,
    416, 626, 42, 271, 359, 163, 508, 229, 576, 147, 560, 35, 714,
    757, 517, 64, 314, 675, 690, 391, 628, 87, 240, 705, 699, 258,
    428, 614, 36, 360, 482, 666, 597, 209, 201, 492, 294]
distance_y = [
    170, 395, 198, 151, 242, 556, 57, 401, 305, 421, 267, 105, 525,
    381, 244, 330, 395, 169, 141, 380, 153, 442, 528, 329, 232, 48,
    498, 265, 343, 120, 165, 50, 433, 63, 491, 275, 348, 222, 288,
    490, 213, 524, 244, 114, 104, 552, 70, 425, 227, 331]
#城市距离和初始信息素
distance_graph = [[0 for col in range(city_num)] for raw in range(city_num)]
print(distance_graph)
pheromone_graph = [[1 for col in range(city_num)] for raw in range(city_num)]

#------------------小蚂蚁好多个，有灵魂，值得用一个类表示----------------------


class Ant():
    def __init__(self, ID):
        self.ID = ID  # 蚂蚁公民怎么能没有身份证呢。
        self.__clean_data()  # 小蚂蚁初始化出生地

    def __clean_data(self):
        self.path = []  # 小蚂蚁的路径
        self.total_distance = 0  # 小蚂蚁当前路径的总距离
        self.move_count = 0  # 小蚂蚁的移动次数
        self.current_city = -1  # 小蚂蚁的当前所在城市
        self.open_table_city = [True for i in range(city_num)]  # 标记哪个城市去过，哪个没去过；也称为禁忌表
        city_index = random.randint(0, city_num-1)  # 随机初始化出生地
        self.current_city = city_index
        self.path.append(city_index)
        self.open_table_city[city_index] = False
        self.move_count = 1

    #小蚂蚁如何选择下一个城市
    def __choice_next_city(self):
        next_city = -1
        select_citys_prob = [0 for i in range(city_num)]  # 去下一个城市的概率
        total_prob = 0

        #小蚂蚁掐指一算，去下一个城市的概率
        for i in range(city_num):
            if self.open_table_city[i]:  # 如果下一个城市没去过，还可以去
                try:
                    #选中概率：与信息素浓度呈正比，与距离呈反比
                    select_citys_prob[i] = pow(pheromone_graph[self.current_city][i], ALPHA) *\
                        pow(1/distance_graph[self.current_city][i], BETA)
                    total_prob += select_citys_prob[i]
                except ZeroDivisionError as e:
                    print('Ant ID: {ID}, current city: {current}, target city: {target}'.
                          format(ID=self.ID, current=self.current_city, target=i))
                    sys.exit(1)

        #勇闯天涯啤酒瓶,轮盘对赌
        if total_prob > 0:
            #产生随机概率
            temp_prob = random.uniform(0, total_prob)
            for i in range(city_num):
                if self.open_table_city[i]:
                    temp_prob -= select_citys_prob[i]
                    if temp_prob < 0:
                        next_city = i
                        break
        #如果没有从轮盘对赌中选出，则顺序选择一个未访问城市
        if next_city == -1:
            for i in range(city_num):
                if self.open_table_city[i]:
                    next_city = i
                    break
        if next_city == -1:
            next_city = random.randint(0, city_num-1)
            while False == self.open_table_city[next_city]:
                next_city = random.randint(0, city_num-1)

        #返回下一个城市序号
        return next_city

    #计算路径总距离
    def __cal_total_distance(self):
        temp_distance = 0
        for i in range(1, city_num):
            start, end = self.path[i], self.path[i-1]
            temp_distance += distance_graph[start][end]

        #回路
        end = self.path[0]
        temp_distance += distance_graph[start][end]
        self.total_distance = temp_distance
    
    #小蚂蚁移动操作
    def __move(self, next_city):
        self.path.append(next_city)
        self.open_table_city[next_city] = False
        self.total_distance += distance_graph[self.current_city][next_city]
        self.current_city = next_city
        self.move_count += 1

    #小蚂蚁踏上寻路之旅
    def search_path(self):
        #初始化数据
        self.__clean_data()

        #搜索路径，遍历完所有城市为止
        while self.move_count < city_num:
            next_city = self.__choice_next_city()
            self.__move(next_city)
        #计算路径总长度
        self.__cal_total_distance()

        # 搜索路径，一直到遍历完所有城市路径

#------------------------------小蚂蚁来解决TSP问题--------------------------------------------------


class TSP():

    def __init__(self, root, width=800, height=600, n=city_num):
        self.root = root
        self.width = width
        self.height = height
        #城市数目
        self.n = n

        #tkinter 来画布
        self.canvas = tkinter.Canvas(
            root, width=self.width, height=self.height, bg="#EBEBEB", xscrollincrement=1, yscrollincrement=1)
        self.canvas.pack(expand=tkinter.YES, fill=tkinter.BOTH)
        self.title("TSP蚁群算法(n:初始化 e:开始搜索 s:停止搜索 q:退出程序)")
        self.__r = 5
        self.__lock = threading.RLock()  # 线程锁

        self.__bindEvents()
        self.new()

    #计算城市之间的距离,欧拉距
    for i in range(city_num):
        for j in range(city_num):
            temp_distance = pow(
                (distance_x[i] - distance_x[j]), 2) + pow((distance_y[i] - distance_y[j]), 2)
            temp_distance = pow(temp_distance, 0.5)
            distance_graph[i][j] = float(int(temp_distance + 0.5))

        # 按键响应程序
    def __bindEvents(self):

        self.root.bind("q", self.quite)  # 退出程序
        self.root.bind("n", self.new)  # 初始化
        self.root.bind("e", self.search_path)  # 开始搜索
        self.root.bind("s", self.stop)  # 停止搜索
# 更改标题

    def title(self, s):
        self.root.title(s)

    # 初始化
    def new(self, evt=None):
        # 停止线程
        self.__lock.acquire()
        self.__running = False
        self.__lock.release()

        self.clear()  # 清除信息
        self.nodes = []  # 节点坐标
        self.nodes2 = []  # 节点对象

        # 初始化城市节点
        for i in range(len(distance_x)):
            # 在画布上随机初始坐标
            x = distance_x[i]
            y = distance_y[i]
            self.nodes.append((x, y))
            # 生成节点椭圆，半径为self.__r
            node = self.canvas.create_oval(x - self.__r,
                                           y - self.__r, x + self.__r, y + self.__r,
                                           fill="#ff0000",  # 填充红色
                                           outline="#000000",  # 轮廓白色
                                           tags="node",
                                           )
            self.nodes2.append(node)
            # 显示坐标
            self.canvas.create_text(x, y - 10,  # 使用create_text方法在坐标（302，77）处绘制文字
                                    text='(' + str(x) + ',' + \
                                    str(y) + ')',  # 所绘制文字的内容
                                    fill='black'  # 所绘制文字的颜色为灰色
                                    )

        # 顺序连接城市
        # self.line(range(city_num))
         # 初始城市之间的距离和信息素
        for i in range(city_num):
            for j in range(city_num):
                pheromone_graph[i][j] = 1.0

        self.ants = [Ant(ID) for ID in range(ant_num)]  # 初始蚁群
        self.best_ant = Ant(-1)  # 初始最优解
        self.best_ant.total_distance = 1 << 31  # 初始最大距离
        self.iter = 1  # 初始化迭代次数

# 将节点按order顺序连线
    def line(self, order):
        # 删除原线
        self.canvas.delete("line")

        def line2(i1, i2):
            p1, p2 = self.nodes[i1], self.nodes[i2]
            self.canvas.create_line(p1, p2, fill="#000000", tags="line")
            return i2

        # order[-1]为初始值
        reduce(line2, order, order[-1])

    # 清除画布
    def clear(self):
        for item in self.canvas.find_all():
            self.canvas.delete(item)

    # 退出程序
    def quite(self, evt):
        self.__lock.acquire()
        self.__running = False
        self.__lock.release()
        self.root.destroy()
        print(u"\n程序已退出...")
        sys.exit()

        # 停止搜索
    def stop(self, evt):
        self.__lock.acquire()
        self.__running = False
        self.__lock.release()

    #开始搜索，小蚂蚁开始行动
        # 开始搜索
    def search_path(self, evt=None):
        # 开启线程
        self.__lock.acquire()
        self.__running = True
        self.__lock.release()

        while self.__running:
            #遍历每一只蚂蚁
            for ant in self.ants:
                ant.search_path()  # 小蚂蚁ant完成一个路径搜索
                #我们的小蚂蚁是不是找到比当前最优的路径了
                if ant.total_distance < self.best_ant.total_distance:
                    #更新最优
                    self.best_ant = copy.deepcopy(ant)
            # 更新信息素
            self.__update_pheromone_gragh()
            print(u"迭代次数：", self.iter, u"最佳路径总距离：",
                  int(self.best_ant.total_distance))
            # 连线
            self.line(self.best_ant.path)
            # 设置标题
            self.title(
                "TSP蚁群算法(n:随机初始 e:开始搜索 s:停止搜索 q:退出程序) 迭代次数: %d" % self.iter)
            # 更新画布
            self.canvas.update()
            self.iter += 1
      # 更新信息素

    def __update_pheromone_gragh(self):

        # 获取每只蚂蚁在其路径上留下的信息素
        temp_pheromone = [[0.0 for col in range(
            city_num)] for raw in range(city_num)]
        for ant in self.ants:
            for i in range(1, city_num):
                start, end = ant.path[i - 1], ant.path[i]
                # 在路径上的每两个相邻城市间留下信息素，与路径总距离反比
                temp_pheromone[start][end] += Q / ant.total_distance
                temp_pheromone[end][start] = temp_pheromone[start][end]

        # 更新所有城市之间的信息素，旧信息素衰减加上新迭代信息素
        for i in range(city_num):
            for j in range(city_num):
                pheromone_graph[i][j] = pheromone_graph[i][j] * \
                    RHO + temp_pheromone[i][j]

    # 主循环
    def mainloop(self):
        self.root.mainloop()


# ----------- 程序的入口处 -----------

if __name__ == '__main__':
    TSP(tkinter.Tk()).mainloop()
