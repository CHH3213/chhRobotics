from utils import *
import numpy as np


class RS:
    def __init__(self) -> None:
        pass
    def CCC(self,x,y,theta):
        """
        三段圆弧组成
        """
        u1,t1 = R(x-sin(theta),y-1+cos(theta))
        if u1**2<=4:
            A = math.asin((u1**2)/4)
            u = M(A+t1)
            u2,t2 = R(x-sin(theta),y-1+cos(theta))
            t = t2
            v = M(theta+t-u)
            L = abs(t)+abs(u)+abs(v)

    def CSC_1(self,x,y,theta):
        """
        两段圆弧与直线组成
        L+S+L+、L−S−L−、R+S+R+、R−S−R−
        """
        u,t = R(x-sin(theta),y-1+cos(theta))
        v = M(theta-t)
        L = abs(t)+abs(u)+abs(v)
    
    def CSC_2(self,x,y,theta):
        """
        两段圆弧与直线组成
        L+S+R+、L−S−R−、R+S+L+、R−S−L−
        """
        u1,t1 = R(x-sin(theta),y-1+cos(theta))
        if u1<=4:
            u = math.sqrt(u1**2-4)
            u2,t2 = R(u,2)
            t = M(t1+t2)
            v = M(t-theta)
            L = abs(t)+abs(u)+abs(v)
        else:
            L=float('inf')
    
    def CCCC_1(self,x,y,theta):
        """四段圆弧组成,第五类基础曲线"""
        xi = x+sin(theta)
        eta = y-1-cos(theta)
        rho1 = (2+math.sqrt(xi**2+eta**2))/4
        u = math.acos(rho1)
        if 0<=rho1<=1 and 0<=u<=math.pi/2 :
            t,v = tauOmega(u,-u,xi,eta,theta)
            L =  abs(t)+2*abs(u)+abs(v)
        else:
            L = float('inf')
    def CCCC_2(self,x,y,theta):
        """四段圆弧组成,第6类基础曲线"""
        xi = x+sin(theta)
        eta = y-1-cos(theta)
        rho2 = (20-xi**2-eta**2)/16
        u = -math.acos(rho2)
        if 0 <= rho2 <= 1 and 0 <= u <= math.pi/2:
            t,v = tauOmega(u,-u,xi,eta,theta)
            L =  abs(t)+2*abs(u)+abs(v)
        else:
            L = float('inf')


    def CCSC__CSCC_1(self,x,y,theta):
        """三段圆弧与一条直线段组成，圆弧路经由两段顺时针方向的曲线构成"""
        xi = x+sin(theta)
        eta = y-1-cos(theta)
        rho,var = R(-eta,xi)
        if rho>=2:
            T,var1 = R(math.sqrt(rho**2-4),-2)
            t = M(var-var1)
            u = 2-var1
            v = M(theta-t-math.pi/2)
            L = abs(t)+abs(u)+abs(v)+math.pi/2
        else:
            L = float('inf')
    
    def CCSC__CSCC_2(self, x, y, theta):
        """三段圆弧与一条直线段组成，由两段逆时针方向的曲线构成"""
        xi = x+sin(theta)
        eta = y-1-cos(theta)
        rho, var = R(-eta, xi)
        
        t = var
        u = 2-rho
        v = M(t+math.pi/2-theta)
        L = abs(t)+abs(u)+abs(v)+math.pi/2
    def CCSCC(self,x,y,theta):
        """
        四段圆弧与一条直线段组成,第二段圆弧与第四段圆弧为π/2圆弧, 中间路径为直线路径, 第二、三、四段圆弧方向与第一段、 第五段路径方向相反
        """
        xi = x+sin(theta)
        eta = y-1-cos(theta)
        rho, var = R(eta, xi)
        if rho>=2:
            t = M(var-math.acos(-2/rho))
            if t<=0:
                L = float('inf')
            else:
                u = 4-(xi+2*cos(t))/sin(t)
                v = M(t-theta)
                L = abs(t)+abs(u)+abs(v)+2*math.pi/2
        else:
            L = float('inf')
            

