import numpy as np
import matplotlib.pyplot as plt
import copy

import path_map as PathMap

'''
copied from https://blog.csdn.net/qq_42727752/article/details/121580147
Astat_NextNode(field, closeList_Sub, nodeSub)：输入地图和closeList_Sub集合和父节点行列位置
# 返回子节点的行列位置矩阵nextNodes_Sub
'''


def Astat_NextNode(field, closeList_Sub, nodeSub):
    # 获取地图尺寸
    rows = len(field)
    cols = len(field[0])
    # 移动方向矩阵
    movePos = [
        [-1, 1], [0, 1], [1, 1], [-1, 0],
        [1, 0], [-1, -1], [0, -1], [1, -1]
        ]
    # 存放子节点行列位置的矩阵
    nextNodes_Sub = []

    for i in range(8):
        r = nodeSub[0] + movePos[i][0]
        c = nodeSub[1] + movePos[i][1]

        # 在地图内
        if -1 < r and r < rows and -1 < c and c < cols:
            # 不为障碍物且不在closeList_Sub内则添加进子节点位置矩阵
            if field[r, c] != 2:
                if [r, c] not in closeList_Sub:
                    nextNodes_Sub.append([r, c])

    return nextNodes_Sub


'''
# 测试地图信息---------------------------------------------------------------
'''

rows = 5
cols = 6
startSub = [2, 1]
goalSub = [2, 5]
obsSub = [[1, 3], [2, 3], [3, 3]]

# 栅格地图属性
field = np.ones((rows, cols))

field[startSub[0], startSub[1]] = 4
field[goalSub[0], goalSub[1]] = 5

for i in range(len(obsSub)):
    field[obsSub[i][0], obsSub[i][1]] = 2

'''
# 最关键一点：=========利用数组下标长度一致，顺序一致的关系，创建表分别存放值和点=====
# openList有四个存储内容,分别记录记录点的行列位置信息，G、H、F值
# 初始时，openList只有起点，closeList为空
'''
openList_Sub = [startSub]
openList_G = [0]
openList_H = [0]
openList_F = [0]

# closeList记录位置信息和距离权值F值，初始化时候为空
closeList_Sub = []
closeList_F = []

# 初始化path，即从起点到地图任意点的路径矩阵
pathSub = []
path = []

# 初始化所有存放路径的矩阵
for c in range(cols+1):
    for r in range(rows+1):
        pathSub.append([r, c])
        # 添加的元素是数组append([])
        path.append([[-1, -1]])

# 对于起点，其路径是已知的，写入起点路径
idx = pathSub.index(startSub)
path[idx] = [startSub]


'''
# ============================开始执行A*算法迭代========================
'''
while True:
    # 1、从openList开始搜索移动代价最小的节点，min函数返回值为[值，位置]
    idx_nodeSub = openList_F.index(min(openList_F))
    nodeSub = openList_Sub[idx_nodeSub]

    # 2、判断是否搜索到终点
    if nodeSub[0] == goalSub[0] and nodeSub[1] == goalSub[1]:
        break

    # ******3、在openList选中最小的F值点作为父节点
    nextNodes_Sub = Astat_NextNode(
        field, closeList_Sub, [nodeSub[0], nodeSub[1]])

    # *******4、判断父节点周围子节点情况，并将子节点依次添加或者更新到openList中
    for i in range(len(nextNodes_Sub)):

        # 需要判断的子节点
        nextSub = nextNodes_Sub[i]

        # 计算代价函数
        g = openList_G[idx_nodeSub] + \
            np.sqrt((nodeSub[0]-nextSub[0])**2+(nodeSub[1]-nextSub[1])**2)
        h = np.abs(goalSub[0]-nextSub[0]) + np.abs(goalSub[1]-nextSub[1])
        f = g + h

        # 判断该子节点是否存在在openList中
        if nextSub in openList_Sub:
            # ******如果存在，则需要比较F值，取F值小的更新F和G、H同时更新路径
            idx_nextSub = openList_Sub.index(nextSub)
            if f < openList_F[idx_nextSub]:
                openList_G[idx_nextSub] = g
                openList_H[idx_nextSub] = h
                openList_F[idx_nextSub] = f

                idx_NextPath = pathSub.index(nextSub)
                idx_NodePath = pathSub.index(nodeSub)
                path[idx_NextPath] = copy.deepcopy(path[idx_NodePath])
                path[idx_NextPath].append(nextSub)

        # *******如果不存在，则添加到openList表中
        else:
            openList_Sub.append(nextSub)
            openList_G.append(g)
            openList_H.append(h)
            openList_F.append(f)

            idx_NextPath = pathSub.index(nextSub)
            idx_NodePath = pathSub.index(nodeSub)
            path[idx_NextPath] = copy.deepcopy(path[idx_NodePath])
            path[idx_NextPath].append(nextSub)

    # 将父节点从openList中移除，添加到closeList中
    closeList_Sub.append(nodeSub)
    closeList_F.append(f)
    openList_Sub.pop(idx_nodeSub)
    openList_G.pop(idx_nodeSub)
    openList_H.pop(idx_nodeSub)
    openList_F.pop(idx_nodeSub)


idx = pathSub.index(goalSub)
optpath = path[idx]

for i in range(len(optpath)):
    field[optpath[i][0], optpath[i][1]] = 6


field[startSub[0], startSub[1]] = 4
field[goalSub[0], goalSub[1]] = 5

PathMap.DrawHeatMap(field)
plt.show()
