"""copied from https://blog.csdn.net/zhj_1121/article/details/107007732

Returns:
    _type_: _description_
"""

# 对象Map，主要有地图数据、起点和终点
class Map(object):
    def __init__(self, mapdata, startx, starty, endx, endy):
        self.data = mapdata
        self.startx = startx
        self.starty = starty
        self.endx = endx
        self.endy = endy


class Node(object):
    # 初始化节点信息
    def __init__(self, x, y, g, h, father):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.father = father

    # 处理边界和障碍点
    def getNeighbor(self, mapdata, endx, endy):
        x = self.x
        y = self.y
        result = []
        # 上方的邻居
        if x != 0 and mapdata[x - 1][y] != 0:
            # 创建上方的邻居子节点，并且计算了实际距离和估计距离。
            upNode = Node(x - 1, y, self.g + 10,
                          (abs(x - 1 - endx) + abs(y - endy)) * 10, self)
            result.append(upNode)
        # 下方的邻居
        if x != len(mapdata) - 1 and mapdata[x + 1][y] != 0:
            downNode = Node(x + 1, y, self.g + 10,
                            (abs(x + 1 - endx) + abs(y - endy)) * 10, self)
            result.append(downNode)
        # 左方的邻居
        if y != 0 and mapdata[x][y - 1] != 0:
            leftNode = Node(x, y - 1, self.g + 10,
                            (abs(x - endx) + abs(y - 1 - endy)) * 10, self)
            result.append(leftNode)
        # 右方的邻居
        if y != len(mapdata[0]) - 1 and mapdata[x][y + 1] != 0:
            rightNode = Node(x, y + 1, self.g + 10,
                             (abs(x - endx) + abs(y + 1 - endy)) * 10, self)
            result.append(rightNode)
        # 西北方的邻居
        if x != 0 and y != 0 and mapdata[x - 1][y - 1] != 0:
            wnNode = Node(x - 1, y - 1, self.g + 14,
                          (abs(x - 1 - endx) + abs(y - 1 - endy)) * 10, self)
            result.append(wnNode)
        # 东北方的邻居
        if x != 0 and y != len(mapdata[0]) - 1 and mapdata[x - 1][y + 1] != 0:
            enNode = Node(x - 1, y + 1, self.g + 14,
                          (abs(x - 1 - endx) + abs(y + 1 - endy)) * 10, self)
            result.append(enNode)
        # 西南方的邻居
        if x != len(mapdata) - 1 and y != 0 and mapdata[x + 1][y - 1] != 0:
            wsNode = Node(x + 1, y - 1, self.g + 14,
                          (abs(x + 1 - endx) + abs(y - 1 - endy)) * 10, self)
            result.append(wsNode)
        # 东南方的邻居
        if x != len(mapdata) - 1 and y != len(mapdata[0]) - 1 and mapdata[x + 1][y + 1] != 0:
            esNode = Node(x + 1, y + 1, self.g + 14,
                          (abs(x + 1 - endx) + abs(y + 1 - endy)) * 10, self)
            result.append(esNode)
        # #如果节点在关闭节点 则不进行处理
        # finaResult = []
        # for i in result:
        #     if(i not in lockList):
        #         finaResult.append(i)
        # result = finaResult
        return result

    # 判断该节点是否在一个列表里
    def hasNode(self, worklist):
        for i in worklist:
            if i.x == self.x and i.y == self.y:
                return True
        return False

    # 在存在的前提下
    def changeG(self, worklist):
        for i in worklist:
            if i.x == self.x and i.y == self.y:
                if i.g > self.g:
                    i.g = self.g


def getKeyforSort(element: Node):
    return element.g + element.h


def astar(workMap):
    startx, starty = workMap.startx, workMap.starty
    endx, endy = workMap.endx, workMap.endy
    startNode = Node(startx, starty, 0, 0, None)
    openList = []
    lockList = []
    lockList.append(startNode)
    currNode = startNode
    while (endx, endy) != (currNode.x, currNode.y):
        workList = currNode.getNeighbor(workMap.data, endx, endy)
        for i in workList:
            if i not in lockList:
                if i.hasNode(openList):
                    i.changeG(openList)
                else:
                    openList.append(i)
        openList.sort(key=getKeyforSort)  # 关键步骤
        currNode = openList.pop(0)
        lockList.append(currNode)
    result = []
    while currNode.father is not None:
        result.append((currNode.x, currNode.y))
        currNode = currNode.father
    result.append((currNode.x, currNode.y))
    return result

# 0代表障碍物，1代表通路
mymap = [
    [1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 0, 1, 1, 1],
    [1, 1, 1, 0, 1, 1, 1],
    [1, 1, 1, 0, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1],
]
map = Map(mymap, 2, 1, 2, 5)
result = astar(map)
result.reverse()
print(result)
