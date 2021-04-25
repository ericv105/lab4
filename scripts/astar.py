#!/usr/bin/env python
# import rospy
import math
import heapq
from collections import deque

def round_reg(x):
    if x - int(x) <= .5:
        return math.floor(x)
    return math.ceil(x)

def h_score(a, b):
    return abs( a[0] - b[0] ) + abs( a[1] - b[1] )

def g_score(start_pos, x):
    return abs( start_pos[0] - x[0] ) + abs( start_pos[1] - x[1] )

def coord_to_arr(x):
    return (round_reg(x[0] + 9), round_reg(-x[1] + 10))
    # return ( round_reg(9 + x[1]), round_reg(10 - x[1]) )

def arr_to_coord(x):
    return (x[0]-8.5, -(x[1]-9.5))

def main():
    map_playground = [
        [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
        [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
        [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
        [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
        [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
        [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
        [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
        [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
        [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
        [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]
    ]
    start = coord_to_arr((-8.0,-2.0))
    # print(start)
    end = coord_to_arr((4.5, 9.0))
    # print(end)

    open_set = []
    closed_set = []
    prev_node = {}
    gdict = {start: 0}
    fdict = {start: h_score(start, end)}

    heapq.heappush(open_set, (fdict[start], start))

    while open_set:
        cur = heapq.heappop(open_set)[1]
        closed_set.append(cur)
        if(cur == end):
            path = deque([])
            while cur in prev_node:
                path.appendleft(arr_to_coord(cur))
                map_playground[cur[1]][cur[0]] = '*'
                cur = prev_node[cur]
            for i in map_playground:
                for j in i:
                    if j == 0:
                        print(' ', end=' ')
                    else:
                        print(j, end=' ')
                # new = [' ' if j == 0 else j for j in i]
                # print('[%s]'%', '.join(map(str, new)))
                print()
            return path
        
        neighbors = [
            (cur[0], cur[1] - 1), # top nbr
            (cur[0] + 1, cur[1]), # right nbr
            (cur[0] - 1, cur[1]), # left nbr
            (cur[0], cur[1] + 1) # bottom nbr
        ]
        for nbr in neighbors:
            tg_score = g_score(start, cur) + h_score(cur, nbr)
            if nbr[0] >= 0 and nbr[0] < len(map_playground[0]):
                if nbr[1] >= 0 and nbr[1] < len(map_playground):
                    if map_playground[nbr[1]][nbr[0]] == 1:
                        continue
                else: continue
            else: continue
            if nbr in closed_set:
                continue
            if tg_score < gdict.get(nbr,0) or nbr not in [i[1] for i in open_set]:
                fdict[nbr] = tg_score + h_score(nbr, end)
                prev_node[nbr] = cur
                gdict[nbr] = tg_score
                # if nbr not in [i[1] for i in open_set]:
                heapq.heappush(open_set, (fdict[nbr], nbr))

if __name__ == "__main__":
    print(main())