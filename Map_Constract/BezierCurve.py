#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2023/10/11 18:20
# @Author : Sunx

"""
绘制贝塞尔曲线,
贝塞尔曲线有控制点控制生成，
n阶贝塞尔曲线，由n+1个控制点控制，
并且符合二项式组合。即，
P = （1-t t）^n P(n-1)
"""
from celluloid import Camera  # 保存动图时用，pip install celluloid
import numpy as np
import math
import matplotlib.pyplot as plt

def BezierNormal(PointList, time):
    """
    :param PointList: 控制点列表
    :param time: 时间变量

    :return: Pt 贝塞尔曲线点
    """
    BezierOrder = len(PointList)
    if BezierOrder == 1:
        return PointList
    BezierOrder = len(PointList) - 1
    Pt = []
    for t in np.arange(0.0, time, 0.01):
        cur = np.array([0, 0])
        for i in range(BezierOrder+1):
            C_n_i = math.factorial(BezierOrder)/(math.factorial(i)*math.factorial(BezierOrder-i))
            cur = cur + C_n_i * (1-t) ** (BezierOrder - i) * (t ** i) * PointList[i]
        Pt.append(cur)
    return Pt

def bezier_curve_res(PointList, time):
    '''
    Args:
        PointList 控制点列表
        time 时间变量
    Returns:
        _type_: 当前t时刻的贝塞尔点
    '''

    n = len(PointList)
    if n == 1:
        return PointList[0]
    BezierOrder = n - 1

    return (1-time) * bezier_curve_res(PointList[0:n-1], time) + (time) * bezier_curve_res(PointList[1:n], time)

def BezierCurveRes(PointsList, time):
    '''
    Args:
        PointsList 控制点列表
        time 时间变量
    Returns:
        Pos 贝塞尔曲线点
    '''
    Pos = []
    for t in np.arange(0, time, 0.01):
        Pos.append(bezier_curve_res(PointsList, t))
    return Pos



def main():
    Ps = np.array([[0, 0], [1, 1], [2, 1], [3, 0], [3, 1]])
    pos = BezierNormal(Ps, 1.0)
    plt.plot(Ps[:, 0], Ps[:, 1])
    print(Ps[:, 0], Ps[:, 1], type(Ps))
    pos = np.array(pos)
    print(pos)
    plt.scatter(pos[:, 0], pos[:, 1], c='r')
    plt.show()

    Pos = BezierCurveRes(Ps, 1)

    print("Pos:",Pos)
    Pos = np.array(Pos)
    plt.plot(Ps[:, 0], Ps[:, 1])
    plt.scatter(Pos[:, 0], Pos[:, 1], c='r')
    plt.pause(0.001)
    plt.show()

if __name__ == "__main__":
    main()