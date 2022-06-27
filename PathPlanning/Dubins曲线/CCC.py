
import numpy as np
import matplotlib.pyplot as plt


"""
Dubins Curve CCC型
"""


#目标定义
#定义起终点[x y psi]
S = np.array([1, 1, 7 * np.pi / 4])
G = np.array([4, 5, 3 * np.pi / 4])
#定义转弯半径
ri = 1
rg = 1
rmid = 1


i = -1 # 1:右转，-1：左转


"""计算首尾圆心坐标及其连线向量V12"""
xi = S[0] + ri * i * np.sin(S[2])
yi = S[1] - ri * i * np.cos(S[2])
xg = G[0] + rg * i * np.sin(G[2])
yg = G[1] - rg * i * np.cos(G[2])

V12 = np.array([xg - xi,yg - yi])
angleV12 = np.arctan(V12[1] / V12[0])

"""计算中间圆坐标及三圆心连线向量V13、V32"""
d12 = np.sqrt((xg - xi) ** 2 + (yg - yi) ** 2)
rmid = np.max(np.array([rmid, (d12 - ri - rg) * 0.5]))
d13 = ri + rmid
d32 = rmid + rg
angleP213 = np.arccos((d12 ** 2 + d13 ** 2 - d32 ** 2) / (2 * d12 * d13)) # 余弦定理
xmid = xi + d13 * np.cos(angleV12 - angleP213)
ymid = yi + d13 * np.sin(angleV12 - angleP213)
V13 = np.array([xmid - xi,ymid - yi])
V32 = np.array([xg - xmid,yg - ymid])

Vn13 = V13 / d13  # 归一化
Vn32 = V32 / d32
"""计算切点坐标"""
xt1 = xi + ri * Vn13[0]
yt1 = yi + ri * Vn13[1]
xt2 = xmid + rmid * Vn32[0]
yt2 = ymid + rmid * Vn32[1]

"""绘图"""
# # 画起终点的初始方向
xiDir = np.array([S[0], S[0]+ri*np.cos(S[2])])
yiDir = np.array([S[1], S[1]+ri*np.sin(S[2])])
xgDir = np.array([G[0], G[0]+rg*np.cos(G[2])])
ygDir = np.array([G[1], G[1]+rg*np.sin(G[2])])

# 画出首尾圆
t = np.arange(0, 2 * np.pi+0.01, 0.01)

circle_xi = xi + ri * np.cos(t)
circle_yi = yi + ri * np.sin(t)

circle_xg = xg + rg * np.cos(t)
circle_yg = yg + rg * np.sin(t)

circle_xmid = xmid + rmid * np.cos(t)
circle_ymid = ymid + rmid * np.sin(t)

#绘图
plt.plot(S[0],S[1],'bo',G[0],G[1],'go',xiDir,yiDir,'-b',xgDir,ygDir,'-g',xi,yi,'k*',xg,yg,'k*',xmid,ymid,'k*',xt1,yt1,'ro',xt2,yt2,'ro',circle_xi,circle_yi,'-r',circle_xg,circle_yg,'-r',circle_xmid,circle_ymid,'-r')
plt.axis('equal')
plt.show()