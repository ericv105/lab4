#!/usr/bin/env python
import rospy
import math
import heapq
from collections import deque
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from time import sleep

def round_reg(x):
    if x - int(x) <= .5:
        return math.floor(x)
    return math.ceil(x)

def h_score(a, b):
    # return math.sqrt( (a[0] - b[0])**2 + (a[1]-b[1])**2 )
    return abs( a[0] - b[0] ) + abs( a[1] - b[1] )

def g_score(start_pos, x):
    # return math.sqrt( (start_pos[0] - x[0])**2 + (start_pos[1]-x[1])**2 )
    return abs( start_pos[0] - x[0] ) + abs( start_pos[1] - x[1] )

def coord_to_arr(x):
    return (round_reg(x[0] + 9), round_reg(-x[1] + 10))

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
    goalx = rospy.get_param("/goalx")
    goaly = rospy.get_param("/goaly")
    start = coord_to_arr((-8.0,-2.0))
    # print(start)
    end = coord_to_arr((goalx, goaly))
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
                path.append(arr_to_coord(cur))
                map_playground[cur[1]][cur[0]] = '*'
                cur = prev_node[cur]
            for i in map_playground:
                for j in i:
                    if j == 0:
                        print(' '),
                    else:
                        print(j),
                print
            return path
        
        neighbors = [
            (int(cur[0]), int(cur[1] - 1)), # top nbr
            (int(cur[0] + 1), int(cur[1])), # right nbr
            (int(cur[0] - 1), int(cur[1])), # left nbr
            (int(cur[0]), int(cur[1] + 1)) # bottom nbr
        ]
        for nbr in neighbors:
            nbr_gscore = g_score(start, cur) + h_score(cur, nbr)
            if nbr[0] >= 0 and nbr[0] < len(map_playground[0]):
                if nbr[1] >= 0 and nbr[1] < len(map_playground):
                    if map_playground[nbr[1]][nbr[0]] == 1:
                        continue
                else: continue
            else: continue
            if nbr in closed_set:
                continue
            if nbr_gscore < gdict.get(nbr,0) or nbr not in [i[1] for i in open_set]:
                fdict[nbr] = nbr_gscore + h_score(nbr, end)
                prev_node[nbr] = cur
                gdict[nbr] = nbr_gscore
                heapq.heappush(open_set, (fdict[nbr], nbr))

if __name__ == "__main__":
    rospy.init_node('talker', anonymous=True)
    path = main()

    pub = rospy.Publisher('path', Path, queue_size=10)
    rate = rospy.Rate(10)
    msg = Path()
    msg.header.frame_id = "pth"
    msg.header.stamp = rospy.Time.now()
    for pt in path:
        pose = PoseStamped()
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.position.z = 0

        msg.poses.append(pose)
    sleep(5)
    pub.publish(msg)
    rospy.spin()