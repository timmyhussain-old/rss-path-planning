#!/usr/bin/env python

import rospy
import numpy as np
import tf
from scipy import signal 
import matplotlib.pyplot as plt
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

        # FIELDS
        #self.goal_pos = None
        #self.start_pos = None
        #self.g_map = None
        #self.map_x_offset = None
        #self.map_y_offset = None
        #self.map_resolution = None

        # Euler angnles:
        #self.yaw = 0
        #self.pitch = 0
        #self.roll = 0
        #self.CCW_rotation_matrix = []
        #self.CW_rotation_matrix = []
        
        # SUBSCRIBERS and PUBLISHERS
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
       

    def map_cb(self, msg):
        data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_x_offset = msg.info.origin.position.x
        self.map_y_offset = msg.info.origin.position.y
        self.map_resolution = msg.info.resolution

        # Obtaining the Euler angles and rotation matrix
        x = msg.info.origin.orientation.x
        y = msg.info.origin.orientation.y
        z = msg.info.origin.orientation.z
        w = msg.info.origin.orientation.w
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion([x,y,z,w])
        self.CCW_rotation_matrix = np.array([[np.cos(self.yaw),-np.sin(self.yaw)],[np.sin(self.yaw),np.cos(self.yaw)]])
        self.CW_rotation_matrix = np.array([[np.cos(self.yaw),np.sin(self.yaw)],[-np.sin(self.yaw),np.cos(self.yaw)]])
        print("THIS IS THE YAW => "+str(self.yaw))
        print("THIS IS THE PITCH => "+str(self.pitch))
        print("THIS IS THE ROLL => "+str(self.roll))

        # Treat some spots as certain obstacles - May not need this
        #self.g_map = data
        self.g_map = np.where(data != 0, 100, data)
        #self.g_map = np.where(self.g_map > 0, 100, self.g_map)

        # Dialating the map using scipy
        kernel = np.array([[1,1,1],[1,1,1],[1,1,1]]*3)
        self.g_map = signal.convolve2d(self.g_map, kernel)

        # VISUALIZING THE MAP - only for debugging
        #plt.imshow(self.g_map)
        #plt.show()
  
    def odom_cb(self, msg):
        x_0 = msg.pose.pose.position.x
        y_0 = msg.pose.pose.position.y
        self.start_pos = (x_0,y_0)

    def goal_cb(self, msg):
        x_0 = msg.pose.position.x
        y_0 = msg.pose.position.y
        self.goal_pos = (x_0,y_0) 
        
        # Only call the path planned once a goal position has been specified
        self.plan_path(self.start_pos, self.goal_pos, self.g_map)

    def plan_path(self, start_position, goal_position, ground_map):
        rospy.loginfo("Building the path...")
        # ADJUSTMENT
        #start
        x_0 = start_position[0]
        y_0 = start_position[1]
        rotated = np.dot(self.CCW_rotation_matrix,np.array([[x_0],[y_0]]))
        x_r = rotated[0][0]
        y_r = rotated[1][0]
        x = (x_r+self.map_x_offset)/self.map_resolution
        y = (y_r+self.map_y_offset)/self.map_resolution
        start_d = (x,y)
        
        #goal
        x_0 = goal_position[0]
        y_0 = goal_position[1]
        rotated = np.dot(self.CCW_rotation_matrix,np.array([[x_0],[y_0]]))
        x_r = rotated[0][0]
        y_r = rotated[1][0]

        x = (x_r+self.map_x_offset)/self.map_resolution
        y = (y_r+self.map_y_offset)/self.map_resolution
        goal_d = (x,y)

        
        start = (int(start_d[0]),int(start_d[1]))
        goal = (int(goal_d[0]), int(goal_d[1]))
        
        #print("start = " + str(start))
        #print("goal = "+ str(goal))

        # Using L-2 norm distance... maybe manhatan would be better?
        def heuristic(location):
            return ((location[0]-goal[0])**2+(location[1]-goal[1])**2)**(1/2)

        # Reconstruct the path using parent pointers
        def build_path(came_from):
            path = []
            # Fix position
            path.append(goal_position)
            position = came_from[goal]
            while came_from[position] is not None:
                x_0 = (position[0]*self.map_resolution)-self.map_x_offset
                y_0 = (position[1]*self.map_resolution)-self.map_y_offset
                rotated = np.dot(self.CW_rotation_matrix,np.array([[x_0],[y_0]]))
                x = rotated[0][0]
                y = rotated[1][0]
                path.append((x,y))
                #path.append(((position[0]*self.map_resolution)+self.map_x_offset,(position[1]*self.map_resolution)+self.map_y_offset))
                position = came_from[position]
            path.append((start_position))
            path.reverse()
            self.trajectory.points = path
            #rospy.loginfo(str(path))
            return
        
        # getting the width and height 
        W = len(ground_map[0])
        H = len(ground_map) 

        # initializing all my sets
        #open_queue = Queue.Queue()
        #open_queue.put(start)
        open_set = set()
        open_set.add(start)
        #closed_set = set() # May not need this
        came_from = {start:None}
        g_score = {start:0}

        f_start = heuristic(start)
        f_score = {start:f_start}
        rev_f_score = {f_start:set()} # just for efficiency
        rev_f_score[f_start].add(start)
        
        all_neighbors = {}
        found = False
        # The main part
        #while not open_queue.empty():
        while len(open_set) != 0:
            #print("Searching...")
            inter = open_set.intersection(rev_f_score[min(rev_f_score)])
            current = inter.pop()
            #rospy.loginfo("Current = " + str(current))
            #current = open_queue.get()
            if current == goal:
                build_path(came_from)
                found = True
                break
            open_set.remove(current)
            rev_f_score[min(rev_f_score)].remove(current)
            if len(rev_f_score[min(rev_f_score)]) == 0:
                del rev_f_score[min(rev_f_score)]

            # Get the neigbnors if they are not already in the `neighbors` dictionary
            neighbors = []
            if current not in all_neighbors:
                # This just looks bad, but it's OK for now
                if current[0]+1 <= W-1:
                    neighbors.append((current[0]+1, current[1]))
                if current[0]-1 >= 0:  
                    neighbors.append((current[0]-1, current[1]))
                if current[1]+1 <= H-1:
                    neighbors.append((current[0], current[1]+1))
                if current[1]-1 >= 0:
                    neighbors.append((current[0], current[1]-1))
                
                #DIAGONALS
                #Bottom right
                if current[0]+1 <= W-1 and current[1]+1 <= H-1:
                    neighbors.append((current[0]+1, current[1]+1))
                #Bottom left
                if current[0]+1 <= W-1 and current[1]-1 >= 0:
                    neighbors.append((current[0]+1, current[1]-1))
                #Top right
                if current[0]-1 >= 0 and current[1]+1 <= H-1:
                    neighbors.append((current[0]-1, current[1]+1))
                #Top left
                if current[0]-1 >= 0 and current[1]-1 <= 0:
                    neighbors.append((current[0]-1, current[1]-1))
                
                all_neighbors[current] = neighbors[:]
            else:
                neighbors = all_neighbors[current][:]

            for each in neighbors:
                tentative_score = g_score[current] + 1
                # Treating anything with probability higher than 10 as a certain wall
                if ground_map[each[1]][each[0]] == 0  and (each not in g_score or tentative_score <= g_score[each]):
                    g_score[each] = tentative_score
                    came_from[each] = current
                    # optimize this part by creating an h = {} and not repeating calculations
                    f_score[each] = g_score[current] + heuristic(each) 
                    if f_score[each] not in rev_f_score:
                        rev_f_score[f_score[each]] = set()
                    rev_f_score[f_score[each]].add(each)
                    if each not in open_set:
                        open_set.add(each)
                        #open_queue.put(each)
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
