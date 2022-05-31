


def dijkstra(matrix, source):
    """迪杰斯特拉算法实现
    Args:
        matrix (_type_): 用邻接矩阵表示带权图
        source (_type_): 起点

    Returns:
        _type_: 最短路径的节点集合，最短路径的节点的最短距离，每个节点到起点的最短路径
    """
    INF = float('inf')
    n = len(matrix)
    m = len(matrix[0])
    assert n == m, "Error, please examine matrix dim"
    assert source < n, "Error, start point should be in the range!"
    S = [source]        # 已找到最短路径的节点集合
    U = [v for v in range(n) if v not in S]  # 记录还未确定最短路径的节点集合
    distance = [INF] * n          # source到已找到最短路径的节点的最短距离
    distance[source] = 0  # 起点到自己的距离
    path_optimal = [[]]*n           # source到其他节点的最短路径
    path_optimal[source] = [source]
    while len(S) < n:   # 当已找到最短路径的节点小于n时
        min_value = INF
        col = -1
        row = -1
        for s in S:     # 以已找到最短路径的节点所在行为搜索对象
            for u in U:   # 从U中搜索尚未记录的节点
                if matrix[s][u] + distance[s] < min_value:  # 找出最小值
                    # 在某行找到最小值要加上source到该行的最短路径
                    min_value = matrix[s][u] + distance[s]
                    row = s         # 记录所在行列
                    col = u
        if col == -1 or row == -1:  # 若没找出最小值且节点还未找完，说明图中存在不连通的节点
            break
        S.append(col)  # 在S中添加已找到的节点
        U.remove(col)  # 从U中移除已找到的节点
        distance[col] = min_value # source到该节点的最短距离即为min_value
        path_optimal[col] = path_optimal[row][:]    # 复制source到已找到节点的上一节点的路径
        path_optimal[col].append(col)       # 再其后添加已找到节点即为source到该节点的最短路径
    return S, distance, path_optimal


def main():
    INF = float('inf')
    # 使用邻接矩阵存储图
    # A B C D E F G
    matrix = [[0, 12, INF, INF, INF, 16, 14],
            [12, 0, 10, INF, INF, 7, INF],
            [INF, 10, 0, 3, 5, 6, INF],
            [INF, INF, 3, 0, 4, INF, INF],
            [INF, INF, 5, 4, 0, 2, 8],
            [16, 7, 6, INF, 2, 0, 9],
            [14, INF, INF, INF, 8, 9, 0]]
    S, distance, path_optimal = dijkstra(matrix, 3)
    print('S:')
    print(S)
    print('distance:')
    print(distance)
    print('path_optimal:')
    for p in path_optimal:
        print(p)

if __name__ == '__main__':
    main()


