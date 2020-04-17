#!/usr/bin/env python

import rospy
import numpy as np
import scipy as sp 
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        
        # Fields
        self.goal_pos = []
        self.start_pos = []
        self.map = None
        
       

    def map_cb(self, msg):
        data = np.array(msg.data)
        # Treat some spots as certain obstacles
        self.map = np.where(data < 0, 100, data)
        self.map = np.where(data > 15, 100, data)
        
        # Dialating the map using scipy
        kernel = np.array([[1,1,1,1],[1,1,1,1],[1,1,1,1],[1,1,1,1]])
        
     

    def odom_cb(self, msg):
        self.start_pos = msg.


    def goal_cb(self, msg):
        pass ## REMOVE AND FILL IN ##

    def plan_path(self, start, goal, ground_map):
        def heuristic(location):
            return ((start[0]-goal[0])**2+(start[1]-goal[1])**2)**(1/2)

        def build_path(came_from):
            path = []
            position = goal
            while came_from[position] is not None:
                path.append(position)
                position = came_from[position]
                position.append(start)
            return path

        # getting the width and height 
        W = len(ground_map[0])
        H = len(ground_map) 

        # initializing all my sets
        open_set = set()
        open_set.add(start)
        closed_set = set()
        came_from = {start:None}
        g_score = {start:0}
        f_score = {start:heuristic(start)}
        rev_f_score = {heuristic(start):start} # just for efficiency... we can fix this later
        all_neighbors = {}

        # The main part
        while len(open_set) > 0:
            current = rev_f_score[min(rev_f_score)]
            print(rev_f_score)
            if current == goal:
                return build_path(came_from)
            print(open_set)
            print(current)
            open_set.remove(current)

            # Get the neigbnors if they are not already in the `neighbors` dictionary
            neighbors = []
            if current not in all_neighbors:
                # This just looks bad, but it's OK for now
                if current[0]+1 <= H-1:
                    neighbors.append((current[0]+1, current[1]))
                if current[0]-1 >= 0:  
                    neighbors.append((current[0]-1, current[1]))
                if current[1]+1 <= W-1:
                    neighbors.append((current[0], current[1]+1))
                if current[1]-1 >= 0:
                    neighbors.append((current[0], current[1]-1))
                # we will ignore diagonals for now
                all_neighbors[current] = neighbors[:]
            else:
                neighbors = all_neighbors[current][:]

            for each in neighbors:
                tentative_score = g_score[current] + 1
                if each not in g_score or (ground_map[current[0]][current[1]] < 1 and tentative_score <= g_score[each]):
                    g_score[each] = tentative_score
                    came_from[each] = current
                    # optimize this part by creating an h = {} and not repeating calculations
                    f_score[each] = g_score[current] + heuristic(each) 
                    rev_f_score[f_score[each]] = each
                    if each not in open_set:
                        open_set.add(each)

        ## CODE FOR PATH PLANNING ##
            
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
