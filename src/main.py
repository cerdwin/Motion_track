#!/usr/bin/env python3
import sys
import math
from copy import deepcopy
from pprint import pprint
 
def ford_fulkerson(graph, start, end, frame1, frame2):
    '''
    actually not the full algorithm, which has been implemented in a different project, in motion tracking like we have
    It suffices to create a primitive initial flow
    :param graph:
    :param start:
    :param end:
    :param frame1:
    :param frame2:
    :return:
    '''
    for k in graph[start]:
        graph[start][k] = (1, 1, 1)
    firsts = frame1
    seconds = frame2
    for x in seconds:
        graph[x][end] = (1, 1, 1)
    for i in range(len(firsts)):
        graph[firsts[i]][seconds[i]] = (0, 1, 1)

    return graph



def eucl(first, second):
    '''
    Works out euclidean distance between two tuples
    :param first:
    :param second:
    :return:
    '''
    return ((abs(first[0])-abs(second[0]))**2+(abs(first[1])-abs(second[1]))**2)**0.5

def residual_graph(graph):
    '''
    Builds a residual graph to a graph on input
    :param graph: A dictionary of tuples of coordinates as keys and dicts as values, where keys are nodes connected
    :return: A residual graph
    '''

    temp_residual_graph = {}
    for k, v in graph.items():
        temp_residual_graph[k] = {}

    for k, v in graph.items():
        for key, value in graph[k].items():
            uf = value[2] - value[1]
            temp_residual_graph[k][key] = (uf, eucl(k, key))
            uf_second = value[1] - value[0]
            temp_residual_graph[key][k] = (uf_second, -eucl(k, key))

    return temp_residual_graph

def bellman_ford(graph):
    old_graph = deepcopy(graph)
    graph = {}
    # 1. Take the current Gf , remove all edges with zero residual capacity (i.e., edges e 2 Ef with uf (e) = 0).
    for key in old_graph:
        graph[key] = {}
        for k in old_graph[key]:
            if old_graph[key][k][0] >0:
                graph[key][k] = old_graph[key][k]

    #2. Introduce a dummy vertex s and connect it with every other vertex in Gf by edge with cost cf (e) = 0.
    graph[(-1,-1)] = {}
    for k, v in graph.items():
        if k != (-1,-1):
            graph[(-1,-1)][k] = (0, 0)

    #3. Compute the shortest path tree (with respect to costs cf ) from the dummy vertex s to all other vertices.
    distances = {}
    for v in graph:
        distances[v] = math.inf
    distances[(-1,-1)] = 0
    predecessors = {}

    # step 2: relax edges repeatedly
    # extract all edges- >[(first, first), (second,second)]
    edges = []
    to_break = True
    for k,v in graph.items():
        for key, value in graph[k].items():
            edges.append([k,key])
    for _ in range(len(graph.keys())):
        for e in edges: # edge in format [(from1, from2), (to1, to2)]
            if distances[e[1]]>distances[e[0]]+graph[e[0]][e[1]][1]:
                distances[e[1]] = distances[e[0]]+graph[e[0]][e[1]][1]
                predecessors[e[1]] = e[0]
                to_break = False
        if to_break:
            break

    # 4. Recover a negative cycle or state that it does not exist.
    counter = 0
    delta = math.inf
    for child, parent in predecessors.items():# starting point
        start = child
        if parent == (-1,-1):
            continue
        C = [(parent, child)]
        current_parent = parent
        while True:
            current_child = current_parent
            current_parent = predecessors[current_child]
            C.append((current_parent, current_child))
            if current_parent == start:
                for pair in C:
                    delta = min(delta, graph[pair[0]][pair[1]][0])
                return delta, C
            if current_parent == (-1,-1):
                break
            if C.count(C[-1])>1:
                #print('we got repetitions')
                break
    print('NO CYCLE DETECTED')
    return 0, None


    # return distances, p
    # 4. Recover a negative cycle or state that it does not exist.
def build_graph(first_frame, second_frame):
    print('first frame:', first_frame)
    print('second frame:', second_frame)
    graph = {}
    # start
    start = (101501011,101501011)
    end = (5956119, 5956119)
    graph[start] = {}
    for i in first_frame:
        graph[start][i] = (1, 0, 1)
    # from first frame to second frame
    for i in first_frame:
        graph[i] = {}
        for j in second_frame:
            graph[i][j] = (0, 0, 1)

    # from second frame to end
    for j in second_frame:
        graph[j] = {}
        graph[j][end] = (1, 0, 1)
    # end
    graph[end] = {}
    return graph
 ##############################################################################



def main_loop(graph):
    Gf = residual_graph(graph) # build residual graph w.r.t. f
    print("Gf is:")
    pprint(Gf)
    print()
    CS = []
    while True:
        delta, C = bellman_ford(Gf)
        print('left bellman...with', C, 'and delta:', delta)
        if C in CS:
            return graph
        CS.append(C)
        if delta >0: #delta is zero only if we fail to find a negative cycle
            for pair in C: # pair is [(parent), (child)]
                #print("we have pair:", pair)
                if pair[1] in graph[pair[0]].keys(): # if edge is a forward arc in G
                    tmp = graph[pair[0]][pair[1]] # (lb, f, ub)
                    tmp = (tmp[0], tmp[1]+delta,tmp[2])
                    graph[pair[0]][pair[1]] = tmp
                else:
                    tmp = graph[pair[1]][pair[0]]
                    tmp = (tmp[0], tmp[1]-delta,tmp[2])
                    graph[pair[1]][pair[0]] = tmp
                # u_f(i,j) <- u_f(i,j)-delta
                tmp = Gf[pair[0]][pair[1]]
                tmp = (tmp[0]-delta, tmp[1])
                Gf[pair[0]][pair[1]] = tmp
                # u_f(j,i) <- u_f(j,i)+delta
                tmp = Gf[pair[1]][pair[0]]
                tmp = (tmp[0]+delta, tmp[1])
                Gf[pair[1]][pair[0]] = tmp
        else:
            print('we are ready to return')
            break
    return graph

if __name__ == "__main__":
    origin = sys.argv[1]
    destination = sys.argv[2]
    d = []
    with open(origin) as fp:
        for number in fp.readline().split():
            d.append(int(number))
        n = d[0]
        p = d[1]
        frames = [[] for _ in range(p)]
        for i in range(p):
            line = [int(imte) for imte in fp.readline().split(' ')]
            for x in  range(2*n):
                if x %2 == 0:
                    frames[i].append((line[x], line[x+1]))

    # c are edges...
    res = []
    debug_mode = False
    for k in range(p-1):
        tmp = []
        for pair in frames[k]:
            base = (-pair[0], -pair[1])
            tmp.append(base)
        frames[k] = tmp
        for i in range(len(frames[k])):
            if frames[k].count(frames[k][i])>1:
                while frames[k].count(frames[k][i])>1:
                    tmp = (frames[k][i][0]-0.0000001, frames[k][i][1]+0.0000001)
                    frames[k][i] = tmp

        for i in range(len(frames[k+1])):
            if frames[k+1].count(frames[k+1][i]) > 1:
                while frames[k+1].count(frames[k+1][i]) > 1:
                    tmp = (frames[k+1][i][0] + 0.0000001, frames[k+1][i][1] - 0.0000001)
                    frames[k+1][i] = tmp


        new_graph = build_graph(frames[k], frames[k+1])
        new_graph = ford_fulkerson(new_graph, (101501011,101501011), (5956119,5956119), frames[k], frames[k+1])

        print('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        new_graph = main_loop(new_graph)
        ret = ''
        for i in range(len(frames[k])):
            for z in range(len(frames[k])):
                if new_graph[frames[k][i]][frames[k+1][z]] == (0, 1, 1):
                    ret+= ' '
                    ret+= str(z+1)
                    break
        res.append(ret[1:])

    pprint(res)
    with open(destination, mode='w') as f:
        for x in range(len(res)):
            f.write(res[x])
            if x != len(res)-1:
                f.write('\n')

