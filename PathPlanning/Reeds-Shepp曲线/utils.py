from math import sin,cos
import math


def M(theta):
    """
    函数 M 用于对2π取模运算，并将弧度值限定在 [−π,π]
    Return the angle phi = theta mod (2 pi) such that -pi <= theta < pi.
    """
    theta = theta % (2*math.pi)
    if theta < -math.pi: return theta + 2*math.pi
    if theta >= math.pi: return theta - 2*math.pi
    return theta

def R(x, y):
    """
    笛卡尔坐标系转极坐标
    Return the polar coordinates (r, theta) of the point (x, y).
    """
    r = math.sqrt(x*x + y*y)
    theta = math.atan2(y, x)
    return r, theta


def tauOmega(u,v0,xi,eta,theta):
    """
    曲线弧度计算函数
    """
    delta = M(u-v0)
    zeta = sin(u)-sin(delta)
    psi = cos(u) - cos(delta) - 1
    t1 = math.atan2(eta*zeta - xi*psi, xi*zeta + eta*psi)
    lam = 2*(cos(delta)-cos(v0)-cos(u))+3
    t = M(t1+math.pi) if lam<0 else M(t1)
    v = M(t-(u-v0)-theta)

    return t,v


def change_of_basis(p1, p2):
    """
    返回以p1为坐标原点的p2转换后的坐标
    Given p1 = (x1, y1, theta1) and p2 = (x2, y2, theta2) represented in a
    coordinate system with origin (0, 0) and rotation 0 (in degrees), return
    the position and rotation of p2 in the coordinate system which origin
    (x1, y1) and rotation theta1.
    """
    theta1 = deg2rad(p1[2])
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    new_x = dx * math.cos(theta1) + dy * math.sin(theta1)
    new_y = -dx * math.sin(theta1) + dy * math.cos(theta1)
    new_theta = p2[2] - p1[2]
    return new_x, new_y, new_theta


def rad2deg(rad):
    """
    弧度制转角度值
    """
    return 180 * rad / math.pi

def deg2rad(deg):
    """
    角度值转弧度制
    """
    return math.pi * deg / 180

def sign(x):
    """
    符号函数
    """
    if x>0:
        return 1
    elif x<0:
        return -1
    else:
        return 0
