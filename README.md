## This is about self-driving!

自动驾驶决策规划控制的理论知识参考[CSDN博客](<https://blog.csdn.net/weixin_42301220/article/details/124832403>) 。

> 推荐结合博客内容来看代码实现。


### 环境依赖

环境使用python3，主要依赖：

- matplotlib
- celluloid
- numpy
- scipy

均可以通过`pip`安装所需要的库。


### 车辆模型

- [车辆运动学模型](https://blog.csdn.net/weixin_42301220/article/details/124747072)
- [运动学模型的线性离散化](https://blog.csdn.net/weixin_42301220/article/details/125032347?spm=1001.2014.3001.5501)
- [车辆动力学模型](https://blog.csdn.net/weixin_42301220/article/details/124776068)
- [动力学横向控制误差模型](https://blog.csdn.net/weixin_42301220/article/details/124836339?spm=1001.2014.3001.5501)

### 坐标系转换

- [Frenet坐标系与Cartesian坐标系（一）](https://blog.csdn.net/weixin_42301220/article/details/125211683)
- [Frenet坐标系与Cartesian坐标系（二）](https://blog.csdn.net/weixin_42301220/article/details/125197484)

### 路径规划

- [全局路径规划算法——Dijkstra算法](https://blog.csdn.net/weixin_42301220/article/details/125060298?spm=1001.2014.3001.5501)
- [全局路径规划算法——蚁群算法](https://blog.csdn.net/weixin_42301220/article/details/125129090?spm=1001.2014.3001.5501)
- [全局路径规划算法——动态规划算法](https://blog.csdn.net/weixin_42301220/article/details/125136221?spm=1001.2014.3001.5501)
- [全局路径规划算法——A*算法](https://blog.csdn.net/weixin_42301220/article/details/125140910?spm=1001.2014.3001.5501)
- [局部路径规划算法——曲线插值法](https://blog.csdn.net/weixin_42301220/article/details/125153270)
- [局部路径规划算法——人工势场法](https://blog.csdn.net/weixin_42301220/article/details/125155505)
- [局部路径规划算法——贝塞尔曲线法](https://blog.csdn.net/weixin_42301220/article/details/125167672)
- [局部路径规划算法——B样条曲线法](https://blog.csdn.net/weixin_42301220/article/details/125173884)
- [局部路径规划算法——DWA算法](https://blog.csdn.net/weixin_42301220/article/details/127769819?spm=1001.2014.3001.5502)
- [基于采样的路径规划算法——PRM](https://blog.csdn.net/weixin_42301220/article/details/125254296)
- [基于采样的路径规划算法——RRT](https://blog.csdn.net/weixin_42301220/article/details/125254061?spm=1001.2014.3001.5501)
- [基于采样的路径规划算法——RRT-Connect](https://blog.csdn.net/weixin_42301220/article/details/125267028?spm=1001.2014.3001.5501)
- [基于采样的路径规划算法——RRT*](https://blog.csdn.net/weixin_42301220/article/details/125275337)
- [路径规划—— Dubins 曲线推导(基于向量的方法)](https://blog.csdn.net/weixin_42301220/article/details/125328823)
- [路径规划—— Dubins 曲线公式总结(基于几何的方法)](https://blog.csdn.net/weixin_42301220/article/details/125493646)
- [路径规划——ReedsShepp 曲线总结](https://blog.csdn.net/weixin_42301220/article/details/125382518)
- [汽车速度规划介绍](https://blog.csdn.net/weixin_42301220/article/details/125831886)

### 决策控制

- [PID实现轨迹跟踪](https://blog.csdn.net/weixin_42301220/article/details/124793474)
- [PurePursuit实现轨迹跟踪](https://blog.csdn.net/weixin_42301220/article/details/124882144?spm=1001.2014.3001.5501)
- [Stanley实现轨迹跟踪](https://blog.csdn.net/weixin_42301220/article/details/124899547)
- [后轮位置反馈实现轨迹跟踪](https://blog.csdn.net/weixin_42301220/article/details/125003918?spm=1001.2014.3001.5501)
- [LQR控制算法](https://blog.csdn.net/weixin_42301220/article/details/124542242)
- [LQR控制实现轨迹跟踪](https://blog.csdn.net/weixin_42301220/article/details/125031348?spm=1001.2014.3001.5501)
- [模型预测控制(MPC)实现轨迹跟踪](https://blog.csdn.net/weixin_42301220/article/details/124566369)

### 预测

- [学习卡尔曼滤波（一）——线性卡尔曼滤波](https://blog.csdn.net/weixin_42301220/article/details/124578094)
- [学习卡尔曼滤波（二）——扩展卡尔曼滤波](https://blog.csdn.net/weixin_42301220/article/details/124605350)
- [学习卡尔曼滤波（三）——无迹卡尔曼滤波](https://blog.csdn.net/weixin_42301220/article/details/124708187)
