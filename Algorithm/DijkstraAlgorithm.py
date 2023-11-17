#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2023/10/11 18:20
# @Author : Sunx

import heapq as hq
import json
from operator import eq
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import math
import time
import random as random
# from celluloid import Camera  # 保存动图时用，pip install celluloid
import datetime
from threading import Thread, Lock
from queue import Queue as Q


class Queue:
    def __init__(self, v, p):  #V is node and p is Priority in a heap tree
        self.v = v
        self.p = p

    def __lt__(self, other):
        return self.p < other.p


def dijkstra(G, S, T):
    '''
    Params:
        G: Graph                                                                                                           #
        S: Source node from where shortest path to be find                                                                 #
        T: Target node till where shortest path to be find  
    Returns:
        path or [] if not find path 
    '''
    start = [Queue(S, 0.0)]                       # Creating initial start node using HeapQ and setting its value to 0.0
    goal = set()                                  # 相当于一个close表
    pred = dict()                                                         # Dictionary to store visited nodes in a graph
    dist = dict()                                                     # Dictionary to store distance from point to point
    pred[S] = None
    dist[S] = 0.0
    while start:
        C = hq.heappop(start).v                   # Pop the smallest item off the heap, maintaining the heap invariant.
        if C == T:
            return traversal(T, pred)
        goal.add(C)
        for pointer in G[C]:
            if pointer in goal:
                continue
            dist_temp = dist[C] + G[C][pointer]['weight']
            if pointer not in dist or dist[pointer] > dist_temp:                          #Checking vertex with low cost
                dist[pointer] = dist_temp
                pred[pointer] = C
                hq.heappush(start, Queue(pointer, dist[C] + G[C][pointer]['weight']))          # Adding vertex to queue
    return []

def dijkstra_s(G, S, T, start, que):
    # Creating initial start node using HeapQ and setting its value to 0.0
    goal = set()                                  # 相当于一个close表
    pred = dict()                                                         # Dictionary to store visited nodes in a graph
    dist = dict()                                                     # Dictionary to store distance from point to point
    pred[S] = None
    dist[S] = 0.0
    while start:
        data = que.get()
        C = hq.heappop(start).v                   # Pop the smallest item off the heap, maintaining the heap invariant.
        if C == T:
            return traversal(T, pred)
        goal.add(C)
        for pointer in G[C]:
            if pointer in goal:
                continue
            dist_temp = dist[C] + G[C][pointer]['weight']
            if pointer not in dist or dist[pointer] > dist_temp:                          #Checking vertex with low cost
                dist[pointer] = dist_temp
                pred[pointer] = C
                hq.heappush(start, Queue(pointer, dist[C] + G[C][pointer]['weight']))          # Adding vertex to queue
        que.put(1)
    return []

def draw_process(graph, path, lp, color, ax):
    '''
    Params: 绘制图像
        graph   
        path
        lp
        color
        ax      
    '''
    pos = nx.circular_layout(graph)
    nx.draw_networkx(graph, pos, node_size=600, ax=ax)
    labels = nx.get_edge_attributes(graph, 'weight')
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=labels, ax=ax)
    nx.draw_networkx_edges(graph, pos, edgelist=path, width=6,alpha=0.5,edge_color=color, style='solid', ax=ax)
    nx.draw_networkx_edges(graph, pos, edgelist=lp, width=6, alpha=0.5, edge_color='r', style='solid', ax=ax)
    plt.show()

def dijkstra_show(G, S, T, graph):
    start = [Queue(S, 0.0)]                       # Creating initial start node using HeapQ and setting its value to 0.0
    goal = set()                                  # 相当于一个close表
    pred = dict()                                                         # Dictionary to store visited nodes in a graph
    dist = dict()                                                     # Dictionary to store distance from point to point
    pred[S] = None
    dist[S] = 0.0
    pos = nx.circular_layout(graph)
    ## 画图
    fig= plt.figure(1)
    ax = fig.add_subplot(1, 1, 1)
    # plt.ylim(-4, 4)
    # camera = Camera(fig)
    point = []
    nx.draw_networkx(graph, pos, node_size=600, ax=ax)
    labels = nx.get_edge_attributes(graph, 'weight')
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=labels)
    while start:
        C = hq.heappop(start).v                   # Pop the smallest item off the heap, maintaining the heap invariant.

        if C == T:
            # lp = []
            # path = traversal(point[-1], pred)
            # for i in range(len(path) - 1):
            #     lp.append([path[i], path[i + 1]])
            # draw_process(graph, [], lp, color='g', ax=ax)
            # animation = camera.animate()
            # animation.save('trajectory.gif')
            return traversal(T, pred)

        goal.add(C)
        point.append(C)
        lp = []
        if len(goal) > 1:
            for i in range(len(point)-1):
                path = traversal(point[-1], pred)
                for i in range(len(path) - 1):
                    lp.append([path[i], path[i + 1]])

        for pointer in G[C]:
            if pointer in goal:
                continue
            dist_temp = dist[C] + G[C][pointer]['weight']

            draw_process(graph, [(C, pointer)], lp, color='g', ax=ax)

            if pointer not in dist or dist[pointer] > dist_temp:                          #Checking vertex with low cost
                dist[pointer] = dist_temp
                pred[pointer] = C
                hq.heappush(start, Queue(pointer, dist[C] + G[C][pointer]['weight']))          # Adding vertex to queue

                draw_process(graph, [(C, pointer)], lp, color='g', ax=ax)

            plt.show()
            plt.pause(2)
    #         camera.snap()
    # animation = camera.animate()
    # animation.save('trajectory.gif')
    return []

def bidirectional_dijkstra(G, S, T):

    startS = [Queue(S, 0.0)]   # Creating initial start node for forward search using HeapQ and setting its value to 0.0
    startT = [Queue(T, 0.0)]   # Creating initial start node for forward search using HeapQ and setting its value to 0.0
    goal_S = set()
    goal_T = set()

    pre_S = dict()
    pre_T = dict()
    dist_S = dict()                                                 # Dictionary to store distance from source to target
    dist_T = dict()                                                 # Dictionary to store distance from target to source

    v_dist = {'weight': math.inf}                                         # Setting other vertex initial distance to inf
    # node = [None] #0
    global node
    node = None

    pre_S[S] = None
    pre_T[T] = None
    dist_S[S] = 0.0
    dist_T[T] = 0.0
    def updateS(v, weight, goal):
        if v in goal:
            distance = dist_T[v] + weight       # 存在一个逻辑问题
            if v_dist['weight'] > distance:
                v_dist['weight'] = distance
                global node
                node = v
                # print("v_dist['weight']:", v_dist['weight'], "node:", node, file=print_log)

    def updateT(v, weight, goal):
        if v in goal:
            distance = dist_S[v] + weight       # 存在一个逻辑问题
            if v_dist['weight'] > distance:
                v_dist['weight'] = distance
                global node
                node = v
                # print("v_dist['weight']:", v_dist['weight'], "node:", node, file=print_log)
                #node[0] = v
    # print_log = open("./logfile.txt", "a", encoding='utf-8')
    # print("\n", file=print_log)
    # print(datetime.datetime.now().strftime('%Y-%m-%d  %H:%M:%S'), file=print_log)
    # print("第{}轮".format(i), file=print_log)
    # print("bidirectional_dijkstra", file=print_log)
    while startS and startT:
        # print("dist_S[startS[0].v]", dist_S[startS[0].v], "dist_T[startT[0].v]", dist_T[startT[0].v],
        #       file=print_log)
        # print("v_dist['weight']", v_dist['weight'], file=print_log)
        if dist_S[startS[0].v] + dist_T[startT[0].v] >= v_dist['weight']:
            return reverse_traversal(node, pre_S, pre_T)

        if len(startS) + len(goal_S) < len(startT) + len(goal_T):
            C = hq.heappop(startS).v                #Pop the smallest item off the heap, maintaining the heap invariant.
            goal_S.add(C)       #C is current node
            # print("C:", C, file=print_log)
            # print("goal_S:", goal_S, file=print_log)
            for fwd in G[C]:
                if fwd in goal_S:
                    continue
                cur_dist = dist_S[C] + G[C][fwd]['weight']
                # print("fwd", fwd, "dist_S[C]:", dist_S[C], "G[C][fwd]['weight']:", G[C][fwd]['weight'], "cur_dist",cur_dist, file=print_log)
                if fwd not in dist_S or cur_dist < dist_S[fwd]:
                    dist_S[fwd] = cur_dist
                    pre_S[fwd] = C
                    hq.heappush(startS, Queue(fwd, cur_dist))
                    #print("startS:", [(i.p, i.v) for i in startS], file=print_log)
                    updateS(fwd, cur_dist, goal_T)
        else:
            C = hq.heappop(startT).v                # Pop the smallest item off the heap, maintaining the heap invariant
            goal_T.add(C)                           # goal_T相当于一个禁闭表
            # print("C:", C, file=print_log)
            # print("goal_T:", goal_T, file=print_log)
            for back in G[C]:                       # 遍历当前节点的所有邻居
                if back in goal_T:                  # 如果邻居在禁闭表中，表明曾经被访问？曾经被访问为什么不能继续访问？避免回路？
                    continue
                cur_dist = dist_T[C] + G[back][C]['weight']             # 计算当起始点到当前节点的邻居的距离
                # print("back", back, "dist_S[C]:", dist_T[C], "G[C][back]['weight']:", G[C][back]['weight'], "cur_dist",
                #       cur_dist, file=print_log)
                if back not in dist_T or cur_dist < dist_T[back]:       # 如果该访问距离小于曾经值更新，
                    dist_T[back] = cur_dist
                    pre_T[back] = C                                     # 记录邻居的父节点
                    hq.heappush(startT, Queue(back, cur_dist))
                #    print("startT:", [(i.p, i.v) for i in startT], file=print_log)
                    updateT(back, cur_dist, goal_S)                      # 更新miu值
    return []


def bidirectional_dijkstra_b(graph, start, target):

    queue_from_start = []
    hq.heappush(queue_from_start, (0.0, start))
    distance_from_start = {node: float('infinity') for node in graph}
    distance_from_start[start] = 0.0
    parents_of_start = {start: None}

    queue_from_target = []
    hq.heappush(queue_from_target, (0.0, target))
    distance_from_target = {node: float('infinity') for node in graph}
    distance_from_target[target] = 0.0
    parents_of_target = {target: None}

    close_of_start = set()          # 访问禁闭表
    close_of_target = set()         # 访问禁闭表

    miu = math.inf
    global node
    node = None
    # 输出到log中，查看如何执行的
    # print_log = open("./logfile.txt", "a", encoding='utf-8')
    # print("\n", file=print_log)
    # print(datetime.datetime.now().strftime('%Y-%m-%d  %H:%M:%S'), file=print_log)
    # print("bidirectional_dijkstra_b", file=print_log)
    # print("第{}轮".format(i), file=print_log)
    while queue_from_start and queue_from_target:
        # print("queue_from_start[0][0] + queue_from_target[0][0]", queue_from_start[0][0], queue_from_target[0][0], file=print_log)
        if queue_from_start[0][0] + queue_from_target[0][0] >= miu:
            return reverse_traversal(node, parents_of_start, parents_of_target)

        cur_dist, cur_node = hq.heappop(queue_from_start)
        close_of_start.add(cur_node)
        # print("cur_dist, cur_node", cur_dist, cur_node, file=print_log)
        # print("close_of_start:", close_of_start, file=print_log)
        for adjacent, weight in graph[cur_node].items():
            if adjacent in close_of_start:
                continue
            distance = cur_dist + weight["weight"]
            # print("adjacent:", adjacent, "weight", weight["weight"], "cur_dist:", distance, file=print_log)
            # print("distance < distance_from_start[adjacent], distacne:", distance, "distance_from_start[adjacent]:",
            #       distance_from_start[adjacent], file=print_log)
            if distance < distance_from_start[adjacent]:
                distance_from_start[adjacent] = distance
                parents_of_start[adjacent] = cur_node
                hq.heappush(queue_from_start, (distance, adjacent))
                # 更新miu值
                if adjacent in close_of_target:
                    dist = distance + distance_from_target[adjacent]
                    if miu > dist:
                        miu = dist
                        node = adjacent
                        # print("miu:", miu, "node:", node, file=print_log)

        cur_dist, cur_node = hq.heappop(queue_from_target)
        # print("cur_dist, cur_node", cur_dist, cur_node, file=print_log)
        close_of_target.add(cur_node)
        # print("close_of_target:", close_of_target, file=print_log)
        for adjacent, weight in graph[cur_node].items():
            if adjacent in close_of_target:
                continue
            distance = cur_dist + weight["weight"]
            # print("adjacent:", adjacent, "weight", weight["weight"], "cur_dist:", distance, file=print_log)
            # print("distance < distance_from_target[adjacent], distacne:", distance,
            #       "distance_from_target[adjacent]:",
            #       distance_from_target[adjacent], file=print_log)
            if distance < distance_from_target[adjacent]:
                distance_from_target[adjacent] = distance
                parents_of_target[adjacent] = cur_node
                hq.heappush(queue_from_target, (distance, adjacent))

                if adjacent in close_of_start:
                    dist = distance + distance_from_start[adjacent]
                    if miu > dist:
                        miu = dist
                        node = adjacent
                        # print("miu:", miu, "node:", node, file=print_log)

    return []


def traversal(T,pred):
    path = []
    while T is not None:
        path.append(T)
        T = pred[T]
    return path[::-1]


def reverse_traversal(v, pre_S, pre_T):
    path = traversal(v, pre_S)
    v = pre_T[v]
    while v is not None:
        path.append(v)
        v = pre_T[v]
    return path


def distance(G, path):
    dist = 0.0
    tot_v = len(path) -1  #Total Number of Vertex minus 1
    for i in range(tot_v):
        dist += G[path[i]][path[i + 1]]['weight']
    return dist


def generate_random_graph(n, e):
    G = nx.dense_gnm_random_graph(n, e)
    for (u, v) in G.edges():
        # G.edge[u][v]['weight'] = random.randint(1, 10)
        G.edges[u, v]['weight'] = random.randint(1, 50)
        # G.edges[u, v]['weight'] = 1.0
        # G.edges(u, v, {'weight': random.randint(1, 10)})
    pos = nx.circular_layout(G)
    nx.draw_networkx(G, pos, node_size=700)
    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    # plt.savefig('graph{}.png'.format(i))
    # plt.show()
    return G

def draw_result(graph, path):
    fig = plt.figure(1)
    ax = fig.add_subplot(1, 1, 1)
    pos = nx.circular_layout(graph)
    nx.draw_networkx(graph, pos, node_size=600, ax=ax)
    labels = nx.get_edge_attributes(graph, 'weight')
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=labels, ax=ax)
    nx.draw_networkx_edges(graph, pos, edgelist=path, width=6,alpha=0.5,edge_color='r',style='solid', ax=ax)
    plt.show()
    plt.savefig('result_graph.png')


def main():
    S = 1  # Source Vertex
    T = 356  # Target Vertex
    n = 5  # No. of nodes
    e = 2  # No. of Edges
    epoch = 100  # 测试次数
    result = dict()
    test_count = 1

    for test_count in range(1, 8):
        print_log = open("./test_result.txt", "a", encoding='utf-8')
        n = int(5 * (10 ** test_count))
        if test_count == 1 :
            e = int((n ** 2) / 2 * random.uniform(0.1, 0.15))
        else:
            e = int(n * math.log(n, 2) * random.uniform(0.9, 1.2))
        S = random.randint(0, n)
        time.sleep(0.005)
        T = random.randint(0, n)
        while eq(S, T):
            T = random.randint(0, n)

        print(datetime.datetime.now().strftime('%Y-%m-%d  %H:%M:%S'), file=print_log)
        print("\n", file=print_log)
        print("测试开始", file=print_log)

        print("测试配置：", file=print_log)
        print("起点S：", S, file=print_log)
        print("终点T：", T, file=print_log)
        print("顶点数n：", n, file=print_log)
        print("边数e：", e, file=print_log)
        print_log.close()
        result_of_dijkstra = []
        result_of_bidijkstra = []
        result_of_bidijkstra_b = []

        for i in range(epoch):
            print_log = open("./test_result.txt", "a", encoding='utf-8')
            print("程序运行：", ((test_count - 1) * 100 + i) / 8, "%")
            print("第{}轮".format(i), file=print_log)
            graph = generate_random_graph(n, e)
            G_to_dict = nx.to_dict_of_dicts(graph)
            # with open("graph.json", "w", encoding='utf-8') as f:
            #     json.dump(G_to_dict, f, indent=4)

            t1 = time.perf_counter()
            path = dijkstra(G_to_dict, S, T)
            t2 = time.perf_counter()
            result_of_dijkstra.append(1000 * (t2 - t1))

            t3 = time.perf_counter()
            bi_path = bidirectional_dijkstra(G_to_dict, S, T, 1)
            t4 = time.perf_counter()
            result_of_bidijkstra.append(1000 * (t4 - t3))

            t5 = time.perf_counter()
            bi_path_b = bidirectional_dijkstra_b(G_to_dict, S, T, 1)
            t6 = time.perf_counter()
            result_of_bidijkstra_b.append(1000 * (t6 - t5))

            # re_path = []
            # for i in range(len(bi_path) - 1):
            #     re_path.append([bi_path[i], bi_path[i + 1]])
            # draw_result(graph, re_path)

            dist = distance(G_to_dict, path)
            bi_dist = distance(G_to_dict, bi_path)
            bi_dist_b = distance(G_to_dict, bi_path_b)

            # output = [["Dijkstra runtime is "+ str(1000 * (t2 - t1))+"milliseconds.\n"],
            #           ["Bi-Dijkstra runtime is "+ str(1000 * (t4 - t3))+"milliseconds.\n"],
            #           ["Bi-Dijkstra-b runtime is "+str(1000 * (t6 - t5))+"milliseconds.\n"]
            #           ]
            # print(output, file=print_log)

            output = [["Dijkstra runtime is " + str(1000 * (t2 - t1)) + "milliseconds.\n"],
                    ["Bi-Dijkstra runtime is " + str(1000 * (t4 - t3)) + "milliseconds.\n"],
                    ["Bi-Dijkstra-b runtime is " + str(1000 * (t6 - t5)) + "milliseconds.\n"],
                    ["Bi Directional Dijkstra " + str(bi_path) + str(bi_dist) + "\n"],
                    ["Bi Directional Dijkstra " + str(bi_path_b) + str(bi_dist_b) + "\n"],
                    ["Dijkstra " + str(path) + str(dist) + "\n"]]
            # print("Bi Directional Dijkstra ", bi_path, bi_dist, file=print_log)
            # print("Bi Directional Dijkstra ", bi_path_b, bi_dist_b, file=print_log)
            # print("Dijkstra", path, dist, file=print_log)
            print(output, file=print_log)
            print("finish")
            print_log.close()

        print_log = open("./test_result.txt", "a", encoding='utf-8')
        result.update({(n, e): {"result_of_dijkstra": np.mean(result_of_dijkstra),
                            "result_of_bidijkstra": np.mean(result_of_bidijkstra),
                            "result_of_bidijkstra_b": np.mean(result_of_bidijkstra_b)}})
        save_result = dict()
        save_result.update({"({}, {})".format(n, e): {"result_of_dijkstra": np.mean(result_of_dijkstra),
                                                  "result_of_bidijkstra": np.mean(result_of_bidijkstra),
                                                  "result_of_bidijkstra_b": np.mean(result_of_bidijkstra_b)}})
        print("result:", file=print_log)
        with open("test_result.json", "a", encoding='utf-8') as f:
            # json.dump("test_count=1", f)
            json.dump(save_result, f, indent=4)
        print("finish", file=print_log)
        print("\n", file=print_log)
        print_log.close()
    # fig = plt.figure(1)
    # ax = fig.add_subplot(1, 1, 1)
    # labels = result.keys()

    draw_picture(result)


def draw_picture(result):
    # 这两行代码解决 plt 中文显示的问题
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False

    fig = plt.figure(1)
    ax = fig.add_subplot(1, 1, 1)

    # 数组方式绘图
    # fig, arr = plt.subplots(4, 1)
    # fig.tight_layout(h_pad=2)

    labels = [i for i in result.keys()]
    mean_result_of_dijkstra = []
    mean_result_of_bidijkstra = []
    mean_result_of_bidijkstra_b = []
    for key in result.keys():
        mean_result_of_dijkstra.append(result[key]["result_of_dijkstra"])
        mean_result_of_bidijkstra.append(result[key]["result_of_bidijkstra"])
        mean_result_of_bidijkstra_b.append(result[key]["result_of_bidijkstra_b"])

    bar_width = 0.3  # 条形宽度
    index_dijkstra = np.arange(len(labels))  # dijkstra条形图的横坐标
    index_bidijkstra = index_dijkstra + bar_width
    index_bidijkstra_b = index_bidijkstra + bar_width

    # for i in range(len(arr)):
    #     ax = arr[i]
    #     ax.bar(index_dijkstra[0], height=mean_result_of_dijkstra[i], width=bar_width, color='yellow', label='dijkstra')
    #     ax.bar(index_bidijkstra[0], height=mean_result_of_bidijkstra[i], width=bar_width, color='orange', label='bidijkstra')
    #     ax.bar(index_bidijkstra_b[0], height=mean_result_of_bidijkstra_b[i], width=bar_width, color='red',
    #             label='bidijkstra_b')
    #     if i == 0:
    #         ax.legend(loc='upper right', bbox_to_anchor=(1.05, 2.2), borderaxespad=0)
    #     ax.set_title(labels[i])
    #     # plt.xticks(index_dijkstra[0] + bar_width / 2, labels)  # 让横坐标轴刻度显示 waters 里的饮用水， index_male + bar_width/2 为横坐标轴刻度的位置
    #     # ax.set_ylabel('平均运行时间 (ms)')  # 纵坐标轴标题
    #     ax.plot()


    # 设置坐标轴刻度
    plt.plot(range(0, ))
    # 使用 bar 函数画出柱状图
    plt.bar(index_dijkstra, height=mean_result_of_dijkstra, width=bar_width, color='yellow', label='dijkstra')
    plt.bar(index_bidijkstra, height=mean_result_of_bidijkstra, width=bar_width, color='orange', label='bidijkstra')
    plt.bar(index_bidijkstra_b, height=mean_result_of_bidijkstra_b, width=bar_width, color='red', label='bidijkstra_b')

    plt.xticks(index_dijkstra + bar_width / 2, labels)  # 让横坐标轴刻度显示 waters 里的饮用水， index_male + bar_width/2 为横坐标轴刻度的位置
    plt.ylabel('平均运行时间 (ms)')  # 纵坐标轴标题
    fig.suptitle('迪杰斯特拉算法之间的比较')  # 图形标题

    plt.legend(loc='upper right', bbox_to_anchor=(1.05, 2.2), borderaxespad=0)  # 显示图例
    plt.subplots_adjust(top=0.85)
    plt.savefig("test_result.png")
    plt.show()





def Graph_Partition(graph):
    upward_graph = dict()
    downward_graph = dict()
    for vet, adjs in graph.items():
        for adj, weight in adjs.items():
            if vet not in upward_graph and adj not in upward_graph.keys():
                upward_graph.update({vet: {adj: weight}})
            elif adj not in upward_graph.keys():
                upward_graph[vet].update({adj: weight})
            elif vet not in upward_graph[adj].keys():
                upward_graph[vet].update({adj: weight})
            else:
                if vet not in downward_graph.keys():
                    downward_graph.update({vet: {adj: weight}})
                else:
                    downward_graph[vet].update({adj: weight})
    with open("upward_graph.json", "w", encoding='utf-8') as f:
        json.dump(upward_graph, f, indent=4)

    with open("downward_graph.json", "w", encoding='utf-8') as f:
        json.dump(downward_graph, f, indent=4)
    return upward_graph, downward_graph


if __name__ == "__main__":
    # main()
    # S = 1  # Source Vertex
    # T = 356  # Target Vertex
    # # n = 5  # No. of nodes
    # # e = 2  # No. of Edges
    # epoch = 100  # 测试次数
    # result = dict()
    # test_count = 5
    #
    # print_log = open("./test_result.txt", "a", encoding='utf-8')
    # n = int(5 * (10 ** test_count))
    # if test_count == 1:
    #     e = int((n ** 2) / 2 * random.uniform(0.1, 0.15))
    # else:
    #     e = int(n * math.log(n, 10) * random.uniform(0.6, 1.0))
    # # S = random.randint(0, n)
    # # time.sleep(0.005)
    # # T = random.randint(0, n)
    # # while eq(S, T):
    # #     T = random.randint(0, n)
    #
    # graph = generate_random_graph(n, e)
    # G_to_dict = nx.to_dict_of_dicts(graph)
    #
    # print(datetime.datetime.now().strftime('%Y-%m-%d  %H:%M:%S'), file=print_log)
    # print("\n", file=print_log)
    # print("测试开始", file=print_log)
    #
    # print("测试配置：", file=print_log)
    # print("起点S：", S, file=print_log)
    # print("终点T：", T, file=print_log)
    # print("顶点数n：", n, file=print_log)
    # print("边数e：", e, file=print_log)
    # print_log.close()
    # result_of_dijkstra = []
    # result_of_bidijkstra = []
    # result_of_bidijkstra_b = []
    #
    # for i in range(epoch):
    #     print_log = open("./test_result.txt", "a", encoding='utf-8')
    #     print("程序运行：", ((test_count - 1) * 100 + i) / 8, "%")
    #     print("第{}轮".format(i), file=print_log)
    #     # with open("graph.json", "w", encoding='utf-8') as f:
    #     #     json.dump(G_to_dict, f, indent=4)
    #
    #     # if i % 50 == 0:
    #     #     graph = generate_random_graph(n, e)
    #     #     G_to_dict = nx.to_dict_of_dicts(graph)
    #
    #     # 随机起点 和 终点  而不是重新更新图
    #     S = random.randint(0, n)
    #     time.sleep(0.005)
    #     T = random.randint(0, n)
    #     while eq(S, T):
    #         T = random.randint(0, n)
    #
    #     t1 = time.perf_counter()
    #     path = dijkstra(G_to_dict, S, T)
    #     t2 = time.perf_counter()
    #     result_of_dijkstra.append(1000 * (t2 - t1))
    #
    #     t3 = time.perf_counter()
    #     bi_path = bidirectional_dijkstra(G_to_dict, S, T, 1)
    #     t4 = time.perf_counter()
    #     result_of_bidijkstra.append(1000 * (t4 - t3))
    #
    #     t5 = time.perf_counter()
    #     bi_path_b = bidirectional_dijkstra_b(G_to_dict, S, T, 1)
    #     t6 = time.perf_counter()
    #     result_of_bidijkstra_b.append(1000 * (t6 - t5))
    #
    #     # re_path = []
    #     # for i in range(len(bi_path) - 1):
    #     #     re_path.append([bi_path[i], bi_path[i + 1]])
    #     # draw_result(graph, re_path)
    #
    #     dist = distance(G_to_dict, path)
    #     bi_dist = distance(G_to_dict, bi_path)
    #     bi_dist_b = distance(G_to_dict, bi_path_b)
    #
    #     # output = [["Dijkstra runtime is "+ str(1000 * (t2 - t1))+"milliseconds.\n"],
    #     #           ["Bi-Dijkstra runtime is "+ str(1000 * (t4 - t3))+"milliseconds.\n"],
    #     #           ["Bi-Dijkstra-b runtime is "+str(1000 * (t6 - t5))+"milliseconds.\n"]
    #     #           ]
    #     # print(output, file=print_log)
    #
    #     output = [["Dijkstra runtime is " + str(1000 * (t2 - t1)) + "milliseconds.\n"],
    #             ["Bi-Dijkstra runtime is " + str(1000 * (t4 - t3)) + "milliseconds.\n"],
    #             ["Bi-Dijkstra-b runtime is " + str(1000 * (t6 - t5)) + "milliseconds.\n"],
    #             ["Bi Directional Dijkstra " + str(bi_path) + str(bi_dist) + "\n"],
    #             ["Bi Directional Dijkstra " + str(bi_path_b) + str(bi_dist_b) + "\n"],
    #             ["Dijkstra " + str(path) + str(dist) + "\n"]]
    #     # print("Bi Directional Dijkstra ", bi_path, bi_dist, file=print_log)
    #     # print("Bi Directional Dijkstra ", bi_path_b, bi_dist_b, file=print_log)
    #     # print("Dijkstra", path, dist, file=print_log)
    #     print(output, file=print_log)
    #     print("finish")
    #     print_log.close()
    #
    # print_log = open("./test_result.txt", "a", encoding='utf-8')
    # result.update({(n, e): {"result_of_dijkstra": np.mean(result_of_dijkstra),
    #                         "result_of_bidijkstra": np.mean(result_of_bidijkstra),
    #                         "result_of_bidijkstra_b": np.mean(result_of_bidijkstra_b)}})
    # save_result = dict()
    # save_result.update({"({}, {})".format(n, e): {"result_of_dijkstra": np.mean(result_of_dijkstra),
    #                                               "result_of_bidijkstra": np.mean(result_of_bidijkstra),
    #                                               "result_of_bidijkstra_b": np.mean(result_of_bidijkstra_b)}})
    # print("result:", file=print_log)
    # with open("test_result.json", "a", encoding='utf-8') as f:
    #     # json.dump("test_count=1", f)
    #     json.dump(save_result, f, indent=4)
    # print("finish", file=print_log)
    # print("\n", file=print_log)
    # print_log.close()
    # labels = []
    # result_of_dijkstra = []
    # result_of_bidijkstra = []
    # result_of_bidijkstra_b = []
    result =    {
                "(50, 168)": {
                "result_of_dijkstra": 0.11171899999993018,
                "result_of_bidijkstra": 0.06721000000001975,
                "result_of_bidijkstra_b": 0.050868000000023894},
                "(500, 4255)": {
                "result_of_dijkstra": 2.0966099999982113,
                "result_of_bidijkstra": 0.5653520000007539,
                "result_of_bidijkstra_b": 0.4472659999981232},
                "(5000, 20324)": {
                "result_of_dijkstra": 20.964367999997183,
                "result_of_bidijkstra": 1.564436999998975,
                "result_of_bidijkstra_b": 2.7817850000066358},
                "(50000, 252959)": {
                "result_of_dijkstra": 381.33580909091131313,
                "result_of_bidijkstra": 7.6978919191719103737,
                "result_of_bidijkstra_b": 24.5950222222153101818}
                }

    draw_picture(result)