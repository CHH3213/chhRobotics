import numpy as np
import math
from time import sleep
from time import time

"""
PID控制器类实现
包括增量式和位置式
"""
# 位置式
class PID_posi:
    """位置式实现1
    """
    def __init__(self, kp, ki, kd, target, upper=1., lower=-1.):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.err = 0
        self.err_last = 0
        self.err_all = 0
        self.target = target
        self.upper = upper
        self.lower = lower
        self.value = 0

    def increase(self, state):
        self.err = self.target - state
        # self.err =state-self.target
        self.value = self.kp * self.err + self.ki * \
            self.err_all + self.kd * (self.err - self.err_last)
        self.update()

    def update(self):
        self.err_last = self.err
        self.err_all = self.err_all + self.err
        if self.value > self.upper:
            self.value = self.upper
        elif self.value < self.lower:
            self.value = self.lower

    def auto_adjust(self, Kpc, Tc):
        self.kp = Kpc * 0.6
        self.ki = self.kp / (0.5 * Tc)
        self.kd = self.kp * (0.125 * Tc)
        return self.kp, self.ki, self.kd

    def set_pid(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def reset(self):
        self.err = 0
        self.err_last = 0
        self.err_all = 0

    def set_target(self, target):
        self.target = target


class PID_posi_2:
    """位置式实现2
    """
    def __init__(self, k=[1., 0., 0.], target=1.0, upper=1.0, lower=-1.0):
        self.kp, self.ki, self.kd = k

        self.e = 0  # error
        self.pre_e = 0  # previous error
        self.sum_e = 0  # sum of error

        self.target = target  # target
        self.upper_bound = upper    # upper bound of output
        self.lower_bound = lower    # lower bound of output

    def set_target(self, target):
        self.target = target

    def set_k(self, k):
        self.kp, self.ki, self.kd = k

    def set_bound(self, upper, lower):
        self.upper_bound = upper
        self.lower_bound = lower

    def cal_output(self, obs):   # calculate output
        self.e = self.target - obs

        u = self.e * self.kp + self.sum_e * \
            self.ki + (self.e - self.pre_e) * self.kd
        if u < self.lower_bound:
            u = self.lower_bound
        elif u > self.upper_bound:
            u = self.upper_bound

        self.pre_e = self.e
        self.sum_e += self.e
        # print(self.sum_e)
        return u

    def reset(self):
        # self.kp = 0
        # self.ki = 0
        # self.kd = 0

        self.e = 0
        self.pre_e = 0
        self.sum_e = 0
        # self.target = 0

    def set_sum_e(self, sum_e):
        self.sum_e = sum_e





# 增量式
class PID_inc:
    """增量式实现
    """
    def __init__(self, kp, ki, kd, target, upper=1., lower=-1.):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.err = 0
        self.err_last = 0
        self.err_ll = 0
        self.target = target
        self.upper = upper
        self.lower = lower
        self.value = 0
        self.inc = 0

    def increase(self, state):
        self.err = self.target - state
        self.inc = self.kp * (self.err - self.err_last) + self.ki * self.err + self.kd * (
            self.err - 2 * self.err_last + self.err_ll)
        self.update()
        return self.value

    def update(self):
        self.err_last = self.err
        self.err_ll = self.err_last
        self.value = self.value + self.inc
        if self.value > self.upper:
            self.value = self.upper
        elif self.value < self.lower:
            self.value = self.lower
