INF = float('INF')
### 状态节点定义
graph = {
    '4': {'D1': {'E': 5}, 'D2': {'E': 2}},
    '3': {'C1': {'D1': 3, 'D2': 9}, 'C2': {'D1': 6, 'D2': 5}, 'C3': {'D1': 8, 'D2': 10}},
    '2': {'B1': {'C1': 12, 'C2': 14, 'C3': 10}, 'B2': {'C1': 6, 'C2': 10, 'C3': 4}, 'B3': {'C1': 13, 'C2': 12, 'C3': 11}},
    '1': {'A': {'B1': 2, 'B2': 5, 'B3': 1}}
    }

### 最优路径及其距离值定义
INF = float('INF')
dists = {
    'A': INF,
    'B1': INF,
    'B2': INF,
    'B3': INF,
    'C1': INF,
    'C2': INF,
    'C3': INF,
    'D1': INF,
    'D2': INF,
    'E': 0
    }

path_opt = {
    'A': ['A'],
    'B1': ['B1'],
    'B2': ['B2'],
    'B3': ['B3'],
    'C1': ['C1'],
    'C2': ['C2'],
    'C3': ['C3'],
    'D1': ['D1'],
    'D2': ['D2'],
    'E': ['E']
}


# 每一个节点的父节点
parents = {
    'A': None,
    'B1': None,
    'B2': None,
    'B3': None,
    'C1': None,
    'C2': None,
    'C3': None,
    'D1': None,
    'D2': None,
    'E': None
    }

# 动态规划函数
def DP(graph, dists, parents):
    for period_key in graph.keys():  # 遍历每一个阶段
        for key_i in graph[period_key].keys():  # 遍历每个阶段的每一个状态节点
            min_key = None
            for key_i_dist in graph[period_key][key_i].keys(): # 遍历当前阶段的每个状态节点到下一阶段的每一条路径
                if graph[period_key][key_i][key_i_dist] + dists[key_i_dist] < dists[key_i]:
                    dists[key_i] = graph[period_key][key_i][key_i_dist] + dists[key_i_dist]
                    parents[key_i] = key_i_dist
                    min_key = key_i_dist  # 找出最小距离值的节点
            path_opt[key_i].extend(path_opt[min_key])  # 将最小距离值的节点添加到最优路径集合



DP(graph, dists, parents)
print("E到每个节点的最短距离：\n",dists)
print("====================")
print("最优时每个节点的父节点：\n",parents)
print("====================")
print("最优路径：\n",path_opt)
