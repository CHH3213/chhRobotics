
import numpy as np
import math
## 递归的方式实现贝塞尔曲线
def bezier(Ps,n,t):
    """递归的方式实现贝塞尔曲线

    Args:
        Ps (_type_): 控制点，格式为numpy数组：array([[x1,y1],[x2,y2],...,[xn,yn]])
        n (_type_): n个控制点，即Ps的第一维度
        t (_type_): 步长t

    Returns:
        _type_: 当前t时刻的贝塞尔点
    """
    if n==1:
        return Ps[0]
    return (1-t)*bezier(Ps[0:n-1],n-1,t)+t*bezier(Ps[1:n],n-1,t)


def bezier_normal(Ps, n, t):
    """普通方式实现贝塞尔曲线

    Args:
        Ps (_type_): 控制点，格式为numpy数组：array([[x1,y1],[x2,y2],...,[xn,yn]])
        n (_type_): n个控制点，即Ps的第一维度
        t (_type_): 时刻t

    Returns:
        _type_: 当前t时刻的贝塞尔点
    """
    if n==1:
        return Ps[0]
    p_t = np.array([0,0])
    n = len(Ps)-1
    for i in range(n+1):
        C_n_i = math.factorial(n)/(math.factorial(i)*math.factorial(n-i))
        p_t =p_t+C_n_i*(1-t)**(n-i)*t**i*Ps[i]
    return p_t
    