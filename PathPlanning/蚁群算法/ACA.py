# ACO算法实现
import numpy as np
import matplotlib.pyplot as plt
"""
有问题
"""
def p2p(Alpha = None,Beta = None): 
    # 1表示障碍物，0表示通路,L*L,L=20
    G = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
            [0,1,1,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0],
            [0,1,1,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0],
            [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,1,0,0],
            [0,1,1,1,0,0,1,1,1,0,0,0,0,0,0,0,1,1,0,0],
            [0,1,1,1,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
            [0,1,1,1,0,0,1,1,1,0,1,1,1,1,0,0,0,1,1,0],
            [0,1,1,1,0,0,0,0,0,0,1,1,1,1,0,0,0,1,1,0],
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
            [0,0,0,0,0,0,0,1,1,1,0,1,1,1,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,1,1,1,0,1,1,1,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,1,1,1,1,0],
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0],
            [1,1,1,1,0,0,0,0,0,0,0,1,1,1,0,1,1,1,1,0],
            [1,1,1,1,0,0,1,1,0,1,1,1,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,1,1,0,1,1,1,0,0,0,0,0,1,1,0],
            [0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,1,1,0],
            [0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0]])
    MM = G.shape[0]
    
    Tau = np.ones((MM * MM,MM * MM)) # 信息素矩阵
    
    Tau = 8.0 * Tau
    K = 100 # 迭代次数（指蚂蚁出动多少波）
    
    M = 50  # 蚂蚁个数
    
    # Alpha=1;                      	   # Alpha 表征信息素重要程度的参数
    # Beta=7;                       	   # Beta 表征启发式因子重要程度的参数
    Rho = 0.3  # Rho 信息素蒸发系数
    
    Q = 1  # Q 信息素增加强度系数
    
    minkl = float('inf')
    
    mink = 0
    minl = 0
    D = G2D(G)
    N = D.shape[0]  # N表示问题的规模（像素个数）
    
    a = 1 # %小方格像素的边长
    
    S = 1  # 最短路径的起始点
    
    E = MM * MM  # 最短路径的目的点
    
    Ex = a * (np.mod(E, MM) - 0.5)  # 终止点横坐标
    
    if Ex == - 0.5:
        Ex = MM - 0.5
    
    Ey = a * (MM + 0.5 - np.ceil(E / MM))  # 终止点纵坐标 ceil向上取整
    
    Eta = np.zeros(N)  # 启发式信息，取为至目标点的直线距离的倒数  N=L^2
    
    #以下启发式信息矩阵,每个点到终点距离的倒数
    for i in range(N):
        ix = a * (np.mod(i,MM) - 0.5)
        if ix == - 0.5:
            ix = MM - 0.5
        iy = a * (MM + 0.5 - np.ceil(i / MM))
        if i != E:
            Eta[i] = 1 / ((ix - Ex) ** 2 + (iy - Ey) ** 2) ** 0.5
        else:
            Eta[i] = 100
    
    ROUTES = np.empty((K, M))  # 用元胞结构存储每一代的每一只蚂蚁的爬行路线，K=100 迭代次数
    
    PL = np.zeros((K,M))  # 用矩阵存储每一代的每一只蚂蚁的爬行路线长度
    
    #启动K轮蚂蚁觅食活动，每轮派出M只蚂蚁
    for k in range(K):
        for m in range(M):
            #状态初始化
            W = S  # 当前节点初始化为起始点 S为起始点
            Path = S  # 爬行路线初始化
            PLkm = 0  # 爬行路线长度初始化
            TABUkm = np.ones(N)  # 未经过表初始化
            TABUkm[S] = 0 # 排除初始点
            DD = D  # 邻接矩阵初始化
            #下一步可以前往的节点
            DW = DD[W,:]
            DW1 = find(DW)  # 当前点8邻域的索引，find：寻找非零元素的索引和值，返回线性索引
            # print(DW)
            for j in range(len(DW1)):
                if TABUkm[DW1[j]] == 0:
                    DW[DW1[j]] = 0
            LJD = find(DW) 
            Len_LJD = len(LJD)
            #蚂蚁未遇到食物或者陷入死胡同或者觅食停止
            while W != E and Len_LJD >= 1:

                #轮盘赌法选择下一步怎么走
                PP = np.zeros((Len_LJD,Len_LJD))
                for i in range(Len_LJD):
                    PP[i] = (Tau[W,LJD[i]] ** Alpha) * ((Eta[LJD[i]]) ** Beta)
                sumpp = sum(PP)
                PP = PP / sumpp
                Pcum = np.zeros(np.shape(PP))
                Pcum[1] = PP[1]
                for i in np.arange(1,Len_LJD):
                    Pcum[i] = Pcum(i - 1) + PP(i)
                Select = find(Pcum >= np.rand())
                to_visit = LJD(Select[1])
                #状态更新和记录
                Path = np.array([Path,to_visit])
                PLkm = PLkm + DD(W,to_visit)
                W = to_visit
                for kk in np.arange(1,N+1).reshape(-1):
                    if TABUkm(kk) == 0:
                        DD[W,kk] = 0
                        DD[kk,W] = 0
                TABUkm[W] = 0
                DW = DD[W,:]
                DW1 = find(DW)
                for j in range(DW1):
                    if TABUkm(DW1[j]) == 0:
                        DW[j] = 0
                LJD = find(DW)
                Len_LJD = len(LJD)

            #记下每一代每一只蚂蚁的觅食路线和路线长度
            ROUTES[k,m] = Path
            if Path[-1] == E:
                PL[k,m] = PLkm
                if PLkm < minkl:
                    mink = k
                    minl = m
                    minkl = PLkm
            else:
                PL[k,m] = 0
        #更新信息素
        Delta_Tau = np.zeros((N,N))
        for m in range(M):
            if PL(k,m):
                ROUT = ROUTES[k,m]
                TS = len(ROUT) - 1
                PL_km = PL(k,m)
                for s in range(TS):
                    x = ROUT(s)
                    y = ROUT(s + 1)
                    Delta_Tau[x,y] = Delta_Tau(x,y) + Q / PL_km
                    Delta_Tau[y,x] = Delta_Tau(y,x) + Q / PL_km
        Tau = np.multiply((1 - Rho),Tau) + Delta_Tau
    
    for i in range(K):
        PLK = PL[i,:]
        Nonzero = find(PLK)
        PLKPLK = PLK(Nonzero)
        if len(PLKPLK) == 0:
            minPL[i] = 10000
            continue
        minPL[i] = np.amin(PLKPLK)
    
    #绘图
    plotif = 1
    
    if plotif == 1:
        minPL = np.zeros((K,K))
        for i in range(K):
            PLK = PL[i,:]
            Nonzero = find(PLK)
            PLKPLK = PLK(Nonzero)
            minPL[i] = np.amin(PLKPLK)
        plt.figure(1)
        plt.plot(minPL)
        plt.hold('on')
        plt.grid('on')
        plt.title('收敛曲线变化趋势')
        plt.xlabel('迭代次数')
        plt.ylabel('最小路径长度')
        plt.figure(2)
        plt.axis(np.array([0,MM,0,MM]))
        for i in np.arange(1,MM+1).reshape(-1):
            for j in np.arange(1,MM+1).reshape(-1):
                if G(i,j) == 1:
                    x1 = j - 1
                    y1 = MM - i
                    x2 = j
                    y2 = MM - i
                    x3 = j
                    y3 = MM - i + 1
                    x4 = j - 1
                    y4 = MM - i + 1
                    np.fill(np.array([x1,x2,x3,x4]),np.array([y1,y2,y3,y4]),np.array([0.2,0.2,0.2]))
                    plt.hold('on')
                else:
                    x1 = j - 1
                    y1 = MM - i
                    x2 = j
                    y2 = MM - i
                    x3 = j
                    y3 = MM - i + 1
                    x4 = j - 1
                    y4 = MM - i + 1
                    np.fill(np.array([x1,x2,x3,x4]),np.array([y1,y2,y3,y4]),np.array([1,1,1]))
                    plt.hold('on')
        plt.hold('on')
        plt.title('本次迭代的最优路径')
        plt.xlabel('x')
        plt.ylabel('y')
        ROUT = ROUTES[mink,minl]
        LENROUT = len(ROUT)
        Rx = ROUT
        Ry = ROUT
        for ii in np.arange(1,LENROUT+1).reshape(-1):
            Rx[ii] = a * (np.mod(ROUT(ii),MM) - 0.5)
            if Rx(ii) == - 0.5:
                Rx[ii] = MM - 0.5
            Ry[ii] = a * (MM + 0.5 - np.ceil(ROUT(ii) / MM))
        plt.plot(Rx,Ry)
    
    
    # G2D函数实现
    
def G2D(G = None): 
    """G是大小为L*L的模拟地图，0代表通路，1代表障碍。D是L^2 * L^2的一个数组，
    存放的是地图G中每个点的八邻域距离信息，每行代表一个点到地图剩余部分的距离信息，
    一共L^2行，即G中L^2个点的八邻域信息，
    因为八邻域最多有8个点，所以D数组里每行至多只有8个非零信息。
    Args:
        G (_type_, optional): _description_. Defaults to None.

    Returns:
        _type_: _description_
    """
    L = G.shape[0]
    D = np.zeros((L * L,L * L))
    for i in range(L):
        for j in range(L):
            if G[i,j] == 0:
                for m in range(L):
                    for n in range(L):
                        if G[m,n] == 0:
                            im = np.abs(i - m)
                            jn = np.abs(j - n)
                            if im + jn == 1 or (im == 1 and jn == 1): # 8邻域内
                                D[(i - 1) * L + j,(m - 1) * L + n] = (im + jn) ** 0.5
    # print(np.shape(D))
    return D
    

def find(condition):
    res = np.nonzero(condition)
    return res



if __name__=="__main__":
    p2p(1,5)