
import numpy as np
import matplotlib.pyplot as plt

"""Dubins Curve CSC型

"""

#目标定义
#定义起终点[x y dir]
S = np.array([1, 1, 7 * np.pi / 4])
G = np.array([6, 8, 3 * np.pi / 4])
#定义转弯半径
ri = 1
rg = 1

#组合

i = -1 # 1:右转，-1：左转 ---起点圆
j = 1 # 1:右转，-1：左转 ---终点圆
k = i*j # 1:RSR/LSL, -1: RSL/LSR

"""计算首尾圆心坐标"""
xi = S[0] + ri * i * np.sin(S[2])
yi = S[1] - ri * i * np.cos(S[2])
xg = G[0] + rg * j * np.sin(G[2])
yg = G[1] - rg * j * np.cos(G[2])

"""计算法向量"""
#起终点圆圆心之间的向量V1=[v1x,v1y]
v1x = xg - xi
v1y = yg - yi
# V1模长
D = np.sqrt(v1x * v1x + v1y * v1y)
# 单位化
v1x = v1x / D
v1y = v1y / D
#计算法向量n
c = (k * ri - rg) / D
nx = v1x * c - j * v1y * np.sqrt(1 - c * c)
ny = v1y * c + j * v1x * np.sqrt(1 - c * c)



"""计算起终点圆的切点"""
xit = xi + k * ri * nx
yit = yi + k * ri * ny
xgt = xg + rg * nx
ygt = yg + rg * ny

# print(xgt-xg,ygt-yg)
# print(nx,ny)

"""绘图"""
# # 画起终点的初始方向
xiDir = np.array([S[0], S[0]+ri*np.cos(S[2])])
yiDir = np.array([S[1], S[1]+ri*np.sin(S[2])])
xgDir = np.array([G[0], G[0]+rg*np.cos(G[2])])
ygDir = np.array([G[1], G[1]+rg*np.sin(G[2])])


#切点连线即切线
tangent_x = np.array([xit, xgt])
tangent_y = np.array([yit, ygt])

# 画出首尾圆
t = np.arange(0, 2 * np.pi+0.01, 0.01)

# t = np.arange(S[2] - np.pi / 2, 3 * np.pi / 2 - np.arctan(ny / nx)+0.01, 0.01)
circle_xi = xi + ri * np.cos(t)
circle_yi = yi + ri * np.sin(t)

# t = np.arange(- np.pi / 2 - np.arctan(ny / nx),G[2] - np.pi / 2+0.01,0.01)
circle_xg = xg + rg * np.cos(t)
circle_yg = yg + rg * np.sin(t)


plt.plot(S[0], S[1], 'bo', G[0], G[1], 'go', xiDir, yiDir, '-b', xgDir, ygDir, '-g', xi, yi,
         'k*', xg, yg, 'k*', circle_xi, circle_yi, '-r', circle_xg, circle_yg, '-r', [xi,xit], [yi,yit], '-k', [xg,xgt],[yg,ygt], '-k', tangent_x, tangent_y, '-r')

plt.axis('equal')
plt.show()
