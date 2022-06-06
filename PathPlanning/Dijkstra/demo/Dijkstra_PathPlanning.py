import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib import colors
import copy

import map


'''
Dijkstra_NextNode(field,node)找寻周围节点路径函数，比MATLAB省略部分多余输入
'''
def Dijkstra_NextNode(field,node):
    rows = len(field)
    cols = len(field[0])
    movepos = [[-1, 1], [0, 1], [1, 1], [-1, 0],
               [1, 0], [-1, -1], [0, -1], [1, -1]]
    nextnodes = [[np.inf,np.inf],[np.inf,np.inf],[np.inf,np.inf],[np.inf,np.inf],
                [np.inf,np.inf],[np.inf,np.inf],[np.inf,np.inf],[np.inf,np.inf]]

    # 将父节点装换成行列式位置信息，方便进行上下左右移动，并对存放子节点的数据进行初始化
    nodeSub = map.ind2sub([rows,cols],node)

    ## 更新父节点朝着各个方向移动到子节点的距离
    for i in range(8):
        r = nodeSub[0] + movepos[i][0]
        c = nodeSub[1] + movepos[i][1]

        # 父节点有效移动范围，不能越界
        if -1 < r and r < rows and -1 < c and c < cols:
            nextnodes[i][0] = map.sub2ind([rows,cols],r,c)

            # 子节点不为障碍物时，计算距离
            if field[r, c] != 2:
                dist = np.sqrt(movepos[i][0]*movepos[i][0]+movepos[i][1]*movepos[i][1])
                nextnodes[i][1] = dist

    return nextnodes

'''
# 初始化地图---------------------------------------------------------------------
'''
rows = 5
cols = 6
startSub = [2,1]
goalSub = [2,5]
obsSub = [[1,3],[2,3],[3,3]]


# 栅格地图属性
field = np.ones((rows, cols))

field[startSub[0], startSub[1]] = 4
field[goalSub[0], goalSub[1]] = 5

for i in range(len(obsSub)):
    field[obsSub[i][0], obsSub[i][1]] = 2

# 数据转换
startIndex = map.sub2ind([rows,cols],startSub[0], startSub[1])
goalIndex = map.sub2ind([rows,cols],goalSub[0], goalSub[1])


'''
## Dijkstra算法******************************************************************************************
# S/U的第一列表示栅格节点线性索引编号
# 对于S，第二列表示从起点到本节点Node已求得的最小距离；
# 对于U，第二列表示从起点到本节点Node暂时求得的最小距离
'''

# U_pos存放索引点
# U_dist存放起点到该点的距离
U_pos = []
U_dist = []

for i in range(rows*cols):
    U_pos.append(i)
    U_dist.append(np.inf)

# 将起点放入S集合，起点到起点距离为0
S_pos = [startIndex]
S_dist = [0]

# 在U集合中删除起点的信息
idx = U_pos.index(startIndex)
U_pos.pop(idx)
U_dist.pop(idx)

# path为n维向量，默认0->rows*cols个，按照索引的一致性，存放点到点的路线
path = []

for i in range(rows*cols):
    # append([数组])
    path.append([-1])




# ======================dijkstra第一段，更新起点的邻节点及代价，目的使得某些变量不为空，进入循环
# 函数DJ_NextNodes主要获得目标点附近8个点的距离值
nextNodes = Dijkstra_NextNode(field,startIndex)

# 起点周围8个点更新在U中的权值，即距离值
for i in range(8):
    nextnode = nextNodes[i][0]

    # 判断该子节点是否存在,然后将子节点权值更新到U
    if nextnode != np.inf:
        idx = U_pos.index(nextnode)
        U_dist[idx] = nextNodes[i][1]

    # 更新起点到周围8个点的路径
    # Tips:起点到任意有效点的路径都有，但是是否为最短路径，通过U集合比较来判断
    # Tips:程序在这一步，并没有进行最短路径的比较
    if nextNodes[i][1] != np.inf:
        path[nextnode] = copy.deepcopy([startIndex,nextNodes[i][0]])


searhx = []
searhy = []
# 新建画布指定大小
fig = plt.figure(figsize=(4,3))
# 新建子图
ax = fig.add_subplot(111)
startXY = map.sub2xy([rows,cols],startSub[0],startSub[1])
goalXY = map.sub2xy([rows,cols],goalSub[0],goalSub[1])
obsX = []
obsY = []

for i in range(len(obsSub)):
    obsxy = map.sub2xy([rows,cols],obsSub[i][0],obsSub[i][1])
    obsX.append(obsxy[0])
    obsY.append(obsxy[1])



# =======================dijkstra第二段，各种数据不为空之后，进行循环遍历
while len(U_pos) > 0:
    # ************求最短路径在U集合找出当前最小距离值的节点，并移除该节点至S集合中
    idx = U_dist.index(min(U_dist))
    dist_min = U_dist[idx]
    node = U_pos[idx]

    S_pos.append(node)
    S_dist.append(dist_min)

    U_pos.pop(idx)
    U_dist.pop(idx)

    # # 绘图1
    nodesub = map.ind2sub([rows,cols],node)
    if field[nodesub[0]][nodesub[1]] == 1:
        nodexy = map.sub2xy([rows,cols],nodesub[0],nodesub[1])
        searhx.append(nodexy[0])
        searhy.append(nodexy[1])


    # 从获得的新节点Node出发（相当于Node = startpos），进行循环判断求最短路径
    nextNodes = Dijkstra_NextNode(field,node)

    # 依次遍历Node（父节点）的邻近节点（子节点），判断子节点是否在U集合中，更新子节点的距离值
    for i in range(8):
        nextnode = nextNodes[i][0]

        if nextnode != np.inf:
            if nextnode not in S_pos:
                idx_u = U_pos.index(nextnode)
                cost = nextNodes[i][1]

                # 判断是否更新，此部分见图解，理解
                # Tips:首先U(:2)初始化默认为inf无穷大，需要一步步的判断更新距离值
                if dist_min+cost < U_dist[idx_u]:
                    U_dist[idx_u] = dist_min+cost
                    path[nextnode] = copy.deepcopy(path[node])
                    path[nextnode].append(nextnode)
    


    # # 绘图2
    plt.cla()
    plt.plot(startXY[0],startXY[1],'r+')
    plt.plot(goalXY[0],goalXY[1],'b+')
    plt.plot(obsX,obsY,'sk')
    plt.plot(searhx,searhy,'sr')
    ax.set_xlim([-1,cols])
    ax.set_ylim([-1,rows])
    ax.set_xticks(np.arange(cols))
    ax.set_yticks(np.arange(rows))
    plt.pause(0.05)
    # plt.show()




opt_pathIndex = path[goalIndex]
optpathsub = []
for i in range(len(opt_pathIndex)):
    optpathsub.append(map.ind2sub([rows,cols],opt_pathIndex[i]))

optx = []
opty = []

for i in range(0,len(optpathsub),1):
    field[optpathsub[i][0]][optpathsub[i][1]] = 6
    optxy = map.sub2xy([rows,cols],optpathsub[i][0],optpathsub[i][1])
    optx.append(optxy[0])
    opty.append(optxy[1])

field[startSub[0], startSub[1]] = 4
field[goalSub[0], goalSub[1]] = 5

plt.figure()
plt.plot(startXY[0],startXY[1],'r+')
plt.plot(goalXY[0],goalXY[1],'b+')
plt.plot(obsX,obsY,'sk')
plt.plot(searhx,searhy,'sr')
plt.plot(optx,opty,'b')
ax.set_xlim([-1,cols])
ax.set_ylim([-1,rows])
ax.set_xticks(np.arange(cols))
ax.set_yticks(np.arange(rows))
# plt.pause(5)
# map.DrawHeatMap(field)
plt.show()

