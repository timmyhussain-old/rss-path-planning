#!/usr/bin/env python

import rospy
import numpy as np
from scipy import signal 
import Queue
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
        # Fields
        self.goal_pos = None
        self.start_pos = None
        self.g_map = None
        
        # Subscribers
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
       

    def map_cb(self, msg):
        data = np.array(msg.data).reshape((msg.info.width, msg.info.height))

       
        
        self.map_x_offset = msg.info.origin.position.x
        self.map_y_offset = msg.info.origin.position.y
        self.map_resolution = msg.info.resolution
        # Treat some spots as certain obstacles
        self.g_map = np.where(data < 0, 100, data)
        self.g_map = np.where(self.g_map > 10, 100, self.g_map)
        
        # Dialating the map using scipy
        kernel = np.array([[1,1,1],[1,1,1],[1,1,1],[1,1,1]])*(1/9)
        self.g_map = signal.convolve2d(sample, ker, boundary='fill', mode='same')
        
        self.x_origin = msg.info.origin.position.x
        self.y_origin = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        
    def odom_cb(self, msg):
        self.start_pos = ((msg.pose.pose.position.x-self.map_x_offset)/self.map_resolution, (msg.pose.pose.position.y-self.map_y_offset)/self.map_resolution)
        #print("start:" + str((msg.pose.pose.position.x, msg.pose.pose.position.y)))

    def goal_cb(self, msg):
        self.goal_pos = ((msg.pose.position.x-self.map_x_offset)/self.map_resolution, (msg.pose.position.y-self.map_y_offset)/self.map_resolution) #(msg.pose.position.x,msg.pose.position.y)
        #print("end: " + str((msg.pose.position.x, msg.pose.position.y)))
        # Only call the path planned once a goal position has been specified
        self.plan_path(self.start_pos, self.goal_pos, self.g_map)

    def plan_path(self, start_position, goal_position, ground_map):
        rospy.loginfo("Building the path...")
        # Making sure that we start from and end at a integer positions
        # TODO: May need to fix this later
        start = (int(start_position[0]),int(start_position[1]))
        goal = (int(goal_position[0]), int(goal_position[1]))
        
        # Using L-2 norm distance... maybe manhatan would be better?
        def heuristic(location):
            return ((start[0]-goal[0])**2+(start[1]-goal[1])**2)**(1/2)

        # Reconstruct the path using parent pointers
        def build_path(came_from):
            path = []

            if goal != goal_position:
                path.append(goal_position)
            position = goal
            while came_from[position] is not None:
                path.append(position)
                position = came_from[position]
            path.append(start)
            if start != start_position:
                path.append(start_position)
            path.reverse()
            
            #fix resolution and add back offset
            path = [((i[0]*self.map_resolution)+self.map_x_offset, (i[1]*self.map_resolution)+self.map_y_offset) for i in path]

            self.trajectory.points = path
            #rospy.loginfo(str(path))
            return
        
        # getting the width and height 
        W = len(ground_map[0])
        H = len(ground_map) 

        # initializing all my sets
        open_queue = Queue.Queue()
        open_queue.put(start)
        open_set = set()
        open_set.add(start)
        #closed_set = set() # May not need this
        came_from = {start:None}
        g_score = {start:0}
        f_score = {start:heuristic(start)}
        rev_f_score = {heuristic(start):start} # Not using this now
        all_neighbors = {}
        found = False
        # The main part
        while not open_queue.empty():
            #current = rev_f_score[min(rev_f_score)]
            current = open_queue.get()
            if current == goal:
                build_path(came_from)
                found = True
                break
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
                
                # DIAGONALS
                #Top right
                if current[0]+1 <= H-1 and current[1]+1 <= W-1:
                    neighbors.append((current[0]+1, current[1]+1))
                #Top left
                if current[0]+1 <= H-1 and current[1]-1 <= W-1:
                    neighbors.append((current[0]+1, current[1]-1))
                #Bottom right
                if current[0]-1 <= H-1 and current[1]+1 <= W-1:
                    neighbors.append((current[0]-1, current[1]+1))
                #Bottom left
                if current[0]-1 <= H-1 and current[1]-1 <= W-1:
                    neighbors.append((current[0]-1, current[1]-1))
                
                all_neighbors[current] = neighbors[:]
            else:
                neighbors = all_neighbors[current][:]

            for each in neighbors:
                tentative_score = g_score[current] + 1
                # Treating anything with probability higher than 10 as a certain wall
                if ground_map[current[0]][current[1]] < 10 and (each not in g_score or tentative_score <= g_score[each]):
                    g_score[each] = tentative_score
                    came_from[each] = current
                    # optimize this part by creating an h = {} and not repeating calculations
                    f_score[each] = g_score[current] + heuristic(each) 
                    rev_f_score[f_score[each]] = each
                    if each not in open_set:
                        open_set.add(each)
                        open_queue.put(each)
        if not found:
            rospy.loginfo("No Path found!")
            return
    
            
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
