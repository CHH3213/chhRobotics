import numpy as np
from math import *


def frenet2Cartesian(s, dot_s, ddot_s, d, dot_d, ddot_d, d_, d__, x_r, y_r, theta_r, k_r, k_r_):
    """Frenet转Cartesian

    Args:
        s (_type_): 为纵向位移，即Frenet纵坐标；
        dot_s (_type_): Frenet纵向速 度
        ddot_s (_type_): Frenet纵向加速度,
        d (_type_): 横向位移, 即Frenet横坐标
        dot_d (_type_): Frenet横向速度
        ddot_d (_type_): Frenet横向加速度
        d_ (_type_): 横向位移对纵向坐标的一阶导数
        d__ (_type_): 横向位移对纵向坐标的二阶导数
        x_r (_type_): 投影点P点在Cartesian坐标系下的x坐标
        y_r (_type_): 投影点P点在Cartesian坐标系下的y坐标
        theta_r (_type_): 投影点P点在Cartesian坐标系下的朝向角
        k_r (_type_): 曲率
        k_r_ (_type_): 曲率对弧长s的一阶导数_

    Returns:
        _type_: _description_
    """
    x = x_r-d*sin(theta_r)
    y = y_r+d*cos(theta_r)
    one_kr_d = 1-k_r*d
    theta_x = theta_r+atan2(d_, one_kr_d)
    delta_theta = theta_x-theta_r
    v_x = sqrt((dot_s*one_kr_d)**2+(dot_s*d_)**2)
    k_x = cos(delta_theta)/one_kr_d*(k_r+(cos(delta_theta)) **2/one_kr_d*(d__+(k_r_*d+k_r*d_)*tan(delta_theta)))
    a_x = ddot_s*one_kr_d/cos(delta_theta)+dot_s**2/cos(delta_theta) * \
        (d_*(k_x*one_kr_d/cos(delta_theta)-k_r)-(k_r_*d+k_r*d_))

    return x, y, theta_x, v_x, a_x, k_x


def cartesian2Frenet(x, y, theta_x, v_x, a_x, k_x, s_r, x_r, y_r, theta_r, k_r, k_r_):
    """全局坐标系转Frenet坐标系

    Args:
        x (_type_): Cartesian坐标系下的车辆横坐标位置
        y (_type_): Cartesian坐标系下的车辆纵坐标位置
        theta_x (_type_): 为方位角，即全局坐标系下的朝向；
        v_x (_type_): Cartesian坐标系下的线速度大小;
        a_x (_type_): Cartesian坐标系下的加速度
        k_x (_type_): 曲率
        s_r (_type_): 投影点的弧长
        x_r (_type_): 投影点P点在Cartesian坐标系下的x坐标
        y_r (_type_): 投影点P点在Cartesian坐标系下的y坐标
        theta_r (_type_): 投影点P点在Cartesian坐标系下的朝向角
        k_r (_type_): 曲率
        k_r_ (_type_): 曲率对弧长s的一阶导数_

    Returns:
        _type_: Frenet坐标系下车辆的运动状态
    """
    delta_theta = theta_x-theta_r
    one_kr_d = 1-k_r*d
    s = s_r
    d = np.sign((y-y_r)*cos(theta_r)-(x-x_r)*sin(theta_r)) * \
        sqrt((x-x_r)**2+(y-y_r)**2)
    dot_d = v_x*sin(delta_theta)
    ddot_d = a_x*sin(delta_theta)
    dot_s = v_x*cos(delta_theta)/one_kr_d
    d_ = one_kr_d*tan(delta_theta)
    d__ = -(k_r_*d+k_r*d_)*tan(delta_theta)+one_kr_d / \
        (cos(delta_theta))**2*(k_x*one_kr_d/cos(delta_theta)-k_r)
    ddot_s = (a_x*cos(delta_theta)-dot_s**2 *
              (d_*(k_x*one_kr_d/cos(delta_theta)-k_r)-(k_r_*d+k_r*d_)))/one_kr_d

    return s, dot_s, ddot_s, d, dot_d, ddot_d, d_, d__
