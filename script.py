#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
from scipy.optimize import leastsq


class Fitter:

    def __init__(self, capacity):

        print("脚本初始化完毕，容量:{}".format(capacity))

        self.capacity = capacity  # 集合容量
        self.firstTime = True  # 是否第一次拟合

        self.speed = []  # buff 角速度
        self.stamps = []  # buff 时间戳
        self.modelParam = [0.785, 1.884, 0, 1.305]  # 大符拟合模型参数

        self.fitParam = np.array([0, 0, 0, 0])  # 大符拟合结果参数
        self.lastFitParam = np.array([0, 0, 0, 0])  # 上一次大符拟合结果参数

    def process(self, stamp, spd,t):
        if len(self.speed) < self.capacity:
            self.speed.append(spd)
            self.stamps.append(stamp)
            print("PY_ATTENTION: 样本数量暂未达到拟合下限，正在添加...")
            return 0
        else:
            print("PY_ATTENTION: 正在拟合...")
            del self.speed[0]
            del self.stamps[0]
            self.speed.append(spd)
            self.stamps.append(stamp)

            try:
                firParam, res = self.fit_sin(self.stamps, self.speed, self.modelParam, t, flag=True)
                print("正弦函数拟合成功，结果为: " + str(firParam[0]) + "sin(" + str(firParam[1]) +
                      "t + " + str(firParam[2]) + ") + " + str(firParam[3]))
                return res
            except:
                print("拟合失败，返回0")
                return 0

    def func_sin(self, x, p):
        a, f, theta, c = p
        return a * np.sin( f * x + theta) + c

    def error_sin(self, p0, x, y):
        return y - self.func_sin(x, p0)

    def fit_sin(self, x, y, p, t, max_fev=1000, flag=False):
        x_sample = np.array(x)
        y_sample = np.array(y)
        x_max = max(x_sample)

        p0 = np.array(p)
        para = leastsq(self.error_sin, p0, args=(x_sample, y_sample), maxfev=max_fev)

        self.fitParam = para[0]
        if (not self.firstTime):
            print(self.lastFitParam)
            self.fitParam = (self.fitParam + self.lastFitParam) / 2
        else:
            self.firstTime = False

        tmp = self.fitParam[0] / ( self.fitParam[1])
        temp_x = x_max + t
        lower = -tmp * np.cos( self.fitParam[1] * x_max + self.fitParam[2]) + self.fitParam[3] * x_max
        upper = -tmp * np.cos( self.fitParam[1] * temp_x + self.fitParam[2]) + self.fitParam[3] * temp_x
        result = round((upper - lower), 3)

        for i in range(4):
            self.fitParam[i] = round(float(self.fitParam[i]), 3)
        self.lastFitParam = self.fitParam

        if flag:
            return list(self.fitParam), result
        else:
            return result


if __name__ == "__main__":
    from matplotlib import pyplot as plt

    # 样本点

    x = [2630.22, 2651.71, 2673.53, 2745.39, 3039.36, 3140.83, 3163.73, 3365.89, 3410.21, 3569.52, 3592.94, 3713.58,
         3996.3, 4205.4, 4261.3, 4307.77, 4406.71, 5292.95, 5361.08, 5430.11, 5480.88, 5630.37, 5816.8, 5861.38,
         6131.23, 6246.7, 6290.38, 6311.75, 6355.52, 6399.71, 6488.82, 6603.15, 6842.06, 6929.15, 7072.23, 7307.86,
         7772.82, 7918.57, 8209.15, 8300.8, 8588.54, 8728.28, 8750.95, 8773.4, 8896.83, 8922.72, 9022.18, 9047.54,
         9210.93, 9259.75, 9309.04, 9357.54, 9521.99, 9763.45, 10624, 10692.7, 10901.8, 11093.3, 11115.8, 11138.9,
         11329.7, 11560.3, 11582.7, 11604.9, 11673.5, 11727.4, 11749.8, 11775.5, 11947.1, 11991.6, 12127.4, 12241.7,
         12493.3, 12695.2, 13015.1, 13294, 13394.3, 13490.8, 13560.6, 13607, 13697.6, 13744, 13765.8, 13787.2, 13808.1,
         14047.5, 14110.9, 14217, 14259.7, 14586.4, 14699.1, 14745.7, 14809.1, 14946.6, 16170.1, 16194.4, 16237.1,
         16258.6, 16431.2, 16452.7, ]
    y = [-0.334864, -0.652081, -1.27506, -1.70741, -2.15144, -1.81518, -2.23097, -2.68792, -2.37827, -2.7759, -2.14108,
         -1.68933, -1.23148, -0.845196, -1.26302, -0.946764, -0.62682, -0.929292, -1.41535, -0.952387, -1.30925,
         -1.77353, -2.49688, -2.10117, -2.40649, -3.08251, -2.54755, -3.15336, -2.46231, -2.96014, -2.17225, -1.83346,
         -1.50257, -1.09878, -0.761926, -0.373246, -0.842166, -1.17315, -1.59632, -2.09498, -2.4125, -3.20812, -2.71147,
         -2.33135, -1.95603, -2.3703, -2.02722, -2.39995, -1.94001, -2.31652, -1.92795, -1.55456, -1.06387, -0.689213,
         -1.05507, -1.4646, -1.88735, -2.44133, -2.08084, -2.44602, -2.13585, -2.51251, -3.09396, -2.58497, -2.06862,
         -2.53533, -2.22543, -1.89601, -2.21716, -1.72003, -1.41408, -1.10535, -0.710681, -0.356343, -0.657582,
         -1.07341, -1.40935, -1.81517, -2.15973, -1.76569, -2.25977, -2.6047, -3.14493, -2.77594, -2.31149, -2.95233,
         -2.48426, -2.89841, -2.07733, -1.76632, -1.39474, -1.76659, -1.43449, -1.05787, -1.511, -1.93872, -2.43502,
         -2.04087, -2.73346, -3.11406]
    x_ = []
    y_ = []
    for i in range(19):
        x_.append(x[i])
        y_.append(y[i])
    x = x_
    y = y_
    print(x)
    print(y)
    x = [a / 1000 for a in x]

    fitter = Fitter(18, 0.5)
    for i in range(len(x)):
        fitter.process_buff(x[i], y[i])

    print("最终拟合参数: {}".format(fitter.fitParam))
    plt.figure(figsize=(12, 8))
    plt.rcParams['font.sans-serif'] = ['simhei']
    plt.rcParams['axes.unicode_minus'] = False

    plt.scatter(x, y, color='red', label="Sample Points")
    print("X-size = {}, Y-size = {}".format(len(x), len(y)))
    x_fit = np.arange(math.ceil(min(x)), math.ceil(max(x) * 1.2), 0.01)
    y_fit = fitter.func_sin(np.array(x_fit), fitter.fitParam)
    x_max = max(x)
    x_min = min(x)
    print("x_min = {}, x_max = {}".format(min(x), max(x)))
    print(x[0], x[18])
    y_bound = np.array([fitter.fitParam[3], fitter.fitParam[3]])
    x_bound = [x_min, x_max]
    plt.plot(x_fit, y_fit, color="orange", label="Fitted Line", linewidth=2)  # 拟合曲线
    plt.plot(x_bound, y_bound, color='gray', linewidth=1, linestyle='--')  # 对称轴
    plt.plot(x_max, fitter.func_sin(x_max, fitter.fitParam), "ob", color='blue', label="current")  # 当前点
    plt.plot(x_max + fitter.t, fitter.func_sin(x_max + fitter.t, fitter.fitParam), "ob", color='purple',
             label='Prediction')  # 预测点
    plt.plot([x_max, x_max], [fitter.func_sin(x_max, fitter.fitParam), fitter.fitParam[3]],
             color='blue', linestyle='--')  # 作最新点到对称轴的垂直虚线
    plt.plot([x_max + fitter.t, x_max + fitter.t],
             [fitter.func_sin(x_max + fitter.t, fitter.fitParam), fitter.fitParam[3]],
             color='purple', linestyle='--')  # 作预测点到对称轴的垂直虚线
    x_series = np.linspace(x_max, x_max + fitter.t, fitter.capacity)  # 积分区域
    plt.fill_between(x_series, fitter.fitParam[3], fitter.func_sin(x_series, fitter.fitParam), facecolor='white',
                     hatch='/',
                     interpolate=True)  # 填充阴影
    plt.xlabel("Time")
    plt.ylabel("Buff spinSpeed")
    plt.legend()
    plt.show()
    exit(0)