#!/usr/bin/env python
import rospy
import math
import heapq
from collections import deque
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from time import sleep
from tf.transformations import euler_from_quaternion



def callback(data):
    # rospy.loginfo(rospy.get_caller_id + "I heard %s", data.data)
    global path
    path = data.poses

def ballback(data):
    global robot_position
    global robot_angle
    
    point = data.pose.pose.position
    robot_position = point

    bot = data.pose.pose.orientation
    robot_angle = euler_from_quaternion([bot.x, bot.y, bot.z, bot.w])[2]

    
def get_distance(r, g):
    return math.sqrt((g.x - r.x)**2 + (g.y - r.y)**2)

def compute_goal_seek_rot(goal_angle):
    if abs(math.degrees(goal_angle)) < 5: return 0
    else: return (goal_angle * 1.0)

def listener():
    rospy.init_node('bug', anonymous=True)
    # path = None
    
    rospy.Subscriber("path", Path, callback)
    rospy.Subscriber("/base_pose_ground_truth", Odometry, ballback)
    global path
    global robot_position
    global robot_angle
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
   
    
    speed = Twist()

    r = rospy.Rate(4)
    sleep(6)
    if path:
        goal = path.pop().pose.position
        while not rospy.is_shutdown():
            goal_dist = get_distance(robot_position, goal)
            print(goal_dist)
            if goal_dist < 0.1:
                goal = path.pop().pose.position

            inc_x = goal.x - robot_position.x
            inc_y = goal.y - robot_position.y

            angle_to_goal = math.atan2(inc_y, inc_x)

            if abs(angle_to_goal - robot_angle) > 0.05:
                speed.linear.x = 0.0
                speed.angular.z = 0.5
            else:
                speed.linear.x = 0.5
                speed.angular.z = 0.0

            pub.publish(speed)
            r.sleep()    
            


# atGoal = False
# goal = Point()
# goal.x = node.pose.position.x
# goal.y = node.pose.position.y
# atg = math.atan2(goal.y - robot_position.y, goal.x - robot_position.x)

# while not atGoal:
#     goal_dist = get_distance(robot_position, goal)
#     print(goal_dist)
#     if goal_dist < .65:
#         atGoal = True
#         vel.linear.x = 0.0
#         vel.angular.z = 0.0
#     elif abs(atg-robot_angle) > 0.01:
#         vel.angular.z = 0.3
#     else:
#         vel.angular.z = 0.0
#         vel.linear.x = 0.5
#     pub.publish(vel)


# while not atGoal:
#                 goal_dist = get_distance(robot_position, goal)
#                 goal_angle = math.atan2(
#                                         goal.y - robot_position.y,
#                                         goal.x - robot_position.x
#                                         )- 2*robot_angle
#                 velocity = Twist()
#                 if(goal_dist < 1.0):
#                     velocity.linear.x = 0
#                     velocity.angular.z = 0
#                     at_goal = True
#                 elif abs(goal_angle - robot_angle) > 0.05:
#                     velocity.linear.x = 0.0
#                     velocity.angular.y = 0.5
#                 else:
#                     velocity.angular = 0.0
#                     velocity.linear.x = 0.5
#                 pub.publish(velocity)

    rospy.spin()

if __name__ == '__main__':
    path = None
    robot_position = Point()
    robot_angle = Pose()
    listener()