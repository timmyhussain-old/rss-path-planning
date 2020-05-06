#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import time
import utils
import tf
import os

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

class PursuitAvoid(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = "/pf/pose/odom"
        self.lookahead        = 40# FILL IN #
        self.speed            = 2.5# FILL IN #
        # self.wrap             = 0# FILL IN #
        self.wheelbase_length = 2.5 # Guess #

        # self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        # self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)

       #**********************************************************************************************************
        # self.path = rospy.get_param("~race_trj")
        rospack = rospkg.RosPack()
        self.lab6_path = rospack.get_path("lab6")
        # self.path = rospy.get_param("~avoid_trj
        self.path = os.path.join(self.lab6_path+"/trajectories/2020-05-06-05-57-52.traj") #fave path 2
        self.path = os.path.join(self.lab6_path+"/trajectories/2020-05-06-08-38-22.traj") #fave path 1 maybe
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.trajectory.load(self.path)
        self.segment_num = max(len(self.trajectory.points), 100)
        # self.segment_num = max(len(self.trajectory.points), 500)

        #Convert trajectory PoseArray to arr (line segment array)
        N = self.segment_num // (len(self.trajectory.points) - 1)
        rospy.loginfo(N)
        self.arr = self.create_segment(self.trajectory.toPoseArray().poses, N)
        # self.arr_flag = 1
        #rospy.loginfo(self.arr)
        #***********************************************************************************************************

        self.drive_pub = rospy.Publisher("/tesse/drive", AckermannDriveStamped, queue_size=1)
        self.drive_msg = AckermannDriveStamped()

        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odomCallback)
        rospy.Subscriber("/tesse/hood_lidar/scan", LaserScan, self.scanCallback)
        # self.segment_num = 1

        # self.arr_flag = 0
        self.odom_flag = 0
        self.scan_flag = 0

        # self.steering_angle_arr = []
        self.previous_steering = 0
        # self.steering_angle_dt = []
        self.time_now = time.time()
        self.obstacle_detected = 0
        self.obstacle_timer = None
        self.avoid_dir_lock = False
        self.avoid_direction = 0
        # rospy.on_shutdown(self.save_arrays)

    # def save_arrays(self):
    #     np.savetxt(os.path.join(self.lab6_path +"/src/steering.txt"), np.array(self.steering_angle_arr))
    #     np.savetxt(os.path.join(self.lab6_path +"/src/steering_dt.txt"), np.array(self.steering_angle_dt))


    def get_index(self, angle1, angle2, angle_min, angle_increment):
        k1 = int ((angle1 - angle_min )/ angle_increment)
        k2 = int ((angle2 - angle_min )/ angle_increment)

        return k1, k2, np.linspace(angle1, angle2, k2-k1, endpoint=True)

    def find_nearest(self, value):
        array = self.angles
        idx = (np.abs(array - value)).argmin()
        return idx

    def get_curvature(self, curvature, msg):
        # curvature_ix = self.find_nearest(np.pi/2)
        # if curvature < 0:
        #     ordered_ix = sorted(self.ix[:curvature_ix], reverse=True) + self.ix[curvature_ix:].tolist()
        # else:
        #     ordered_ix = self.ix[curvature_ix:].tolist() + sorted(self.ix[:curvature_ix], reverse=True)
        # for i in ordered_ix:
        #     ix1, ix2, front_angs = self.get_index(self.angles_minus[i], self.angles_plus[i], msg.angle_min, msg.angle_increment)
        #     # rospy.loginfo("ix1: " + str(ix1) + " ix2: " + str(ix2) + "\n")
        #     f = np.poly1d(np.polyfit(front_angs, msg.ranges[ix1:ix2], deg=1))
        #     low_mean_min = np.mean(sorted(f(front_angs))[:(ix2-ix1)//2])
        #     rospy.loginfo(str(self.angles[i]) + "/" + str(np.rad2deg(self.angles[i]))+ ": " + str(low_mean_min))
        #     print('---------------------------------------------------')
        #     # time.sleep(1)
        #     if low_mean_min > 10: #self.lookahead:
        #         # print(self.angles[i])
        #         break
            # else:
            #     ix1, ix2, front_angs = self.get_index(-self.angles_plus[i], -self.angles_minus[i], msg.angle_min, msg.angle_increment)
            #     A = np.array([front_angs, [1]*len(front_angs)]).T
            #     f = np.poly1d(np.polyfit(front_angs, msg.ranges[ix1:ix2], deg=1))
            #     low_mean_min = np.mean(sorted(f(front_angs))[:(ix2-ix1)//2])
            #     # rospy.loginfo("ix1: " + str(ix1) + " ix2: " + str(ix2) + "\n")
            #     if low_mean_min > 10: #self.lookahead:
            #         break
        # rospy.loginfo(low_mean_min)

        ix1, ix2, front_angs = self.get_index(np.deg2rad(-30), np.deg2rad(30), msg.angle_min, msg.angle_increment)
        # rospy.loginfo("ix1: " + str(ix1) + " ix2: " + str(ix2) + "\n")
        z = np.polyfit(front_angs, msg.ranges[ix1:ix2], deg=1)
        f = np.poly1d(z)
        front_arr = f(front_angs)
        front_thin = np.array_split(front_arr, 10)
        front_thin = np.min([np.mean(i) for i in front_thin])
        low_mean_min = np.mean(sorted(front_arr)[:(ix2-ix1)//6])

        ix1, ix2, front_angs = self.get_index(np.deg2rad(-50), np.deg2rad(-25), msg.angle_min, msg.angle_increment)
        z = np.polyfit(front_angs, msg.ranges[ix1:ix2], deg=1)
        fleft = np.poly1d(z)
        low_mean_left = np.mean(sorted(fleft(front_angs))[:(ix2-ix1)//6])

        ix1, ix2, front_angs = self.get_index(np.deg2rad(25), np.deg2rad(50), msg.angle_min, msg.angle_increment)
        z = np.polyfit(front_angs, msg.ranges[ix1:ix2], deg=1)
        fright = np.poly1d(z)
        low_mean_right = np.mean(sorted(fright(front_angs))[:(ix2-ix1)//6])

        rospy.loginfo(low_mean_min)
        speed = 15.0
        if self.speed > 10.0:
            zero_thresh, side_thresh = 30, 8

        elif self.speed < 10.0:
            zero_thresh, side_thresh = 25, 8
        else:
            zero_thresh, side_thresh = 10, 5
        if (front_thin + self.speed * 1.5 < zero_thresh) or ((curvature < 0)*low_mean_left + (curvature > 0)*low_mean_right < side_thresh):

            speed = 5.0

            r = zero_thresh * 1.3
            x_body = r*np.cos(np.deg2rad(40))
            # y_body = r*np.sin(np.deg2rad(30))
            R_track = r**2/(2*x_body)
            # if self.correction:
            #     curvature = np.sign(curvature) * np.arctan(self.wheelbase_length/R_track) #- np.pi/2
            # elif np.sign(z[0]) == np.sign(curvature):
            #     curvature = np.sign(z[0]) * np.arctan(self.wheelbase_length/R_track) #- np.pi/2
            #     self.correction = 0
            # else:


            if np.random.random() > 0.9:
                self.avoid_direction = ((low_mean_right > low_mean_left)*-1 + (low_mean_right < low_mean_left)*1)
            else:
                self.avoid_direction = np.sign(curvature)
            #     self.obstacle_detected = 1
            #     self.obstacle_timer = time.time()
            curvature =  self.avoid_direction * np.arctan(self.wheelbase_length/R_track)
            if front_thin < 1.0:
                # speed = 1.0
                speed = 1.0
                curvature = 2*curvature
            # if (time.time() - self.obstacle_timer > 2.0):
            #     self.obstacle_detected = 0
                # self.correction =1
            # del R_track, x_body, y_body
        return speed, curvature

    def scanCallback(self, msg):
        ranges = np.array(msg.ranges)
        if not self.scan_flag:
            # angle = np.arctan(1/self.lookahead)
            angle = np.deg2rad(40)
            self.angles = np.linspace(msg.angle_min+angle, msg.angle_max - angle, 50)
            self.angles_plus = self.angles + angle
            self.angles_minus = self.angles - angle
            self.ix = np.arange(len(self.angles))
            del angle
            self.scan_flag = 1

        curvature = None

        if self.odom_flag:
            ix_min = self.find_closest_point(self.arr, self.loc) #Index of closest line segment on trajectory to the car
            try:
        	#rospy.loginfo('Start math')
        	#rospy.loginfo(ix_min)
                curvature = self.find_lookahead(self.arr, ix_min, self.loc, self.quat, self.lookahead) #curvature (steering angle) in global frame
                # self.drive_msg.drive.speed = self.speed
                # self.drive_msg.drive.steering_angle = curvature
                # self.drive_pub.publish(self.drive_msg)
                #rospy.loginfo('Published drive')

            except UnboundLocalError:
                #
                # curvature = None
                print("failed")
                pass


        if curvature is not None:
            # if abs(curvature) > 0.1:
            #     self.lookahead = max(self.lookahead**1/1.3, 5)
            # else:
            #     self.lookahead = min(self.lookahead**1.3, 40)
            # rospy.loginfo("initial curvature: " + str(curvature))



            # self.get_curvature(curvature, msg)
            # self.steering_angle_arr.append(curvature)

            # self.steering_angle_dt.append((curvature - self.previous_steering) / dt)
            delt = 0.2
            if time.time()- self.time_now > delt:
                dt = time.clock()-self.time_now
                speed, curvature = self.get_curvature(curvature, msg)
                cmdangle = 0.1*curvature +  0.04*(curvature - self.previous_steering)/dt

                self.drive_msg.drive.speed = speed #self.speed
                # rospy.loginfo("curvature: " + str(curvature))
                #
                # rospy.loginfo(1.8*(curvature - self.previous_steering)/delt)
                # rospy.loginfo('------------------------------------')
                self.drive_msg.drive.steering_angle = cmdangle
                # self.drive_msg.drive.steering_angle = 0 #1.8*(curvature - self.previous_steering)/delt #0.8*curvature + 0.017 *(curvature - self.previous_steering) /dt
                self.drive_pub.publish(self.drive_msg)
                self.previous_steering = self.drive_msg.drive.steering_angle
                self.time_now = time.time()
        else:
            # self.lookahead = self.lookahead**1.2
            self.drive_msg.drive.speed = 0.0
            self.drive_msg.drive.steering_angle = 0
            self.drive_pub.publish(self.drive_msg)


    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        rospy.loginfo('Start Traj callback')

        #print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.segment_num = max(len(msg.poses), 500)

        #Convert trajectory PoseArray to arr (line segment array)
        N = self.segment_num // (len(msg.poses) - 1)
        rospy.loginfo(N)
        self.arr = self.create_segment(msg.poses, N)
        self.arr_flag = 1
        rospy.loginfo(self.arr)

    def odomCallback(self, msg):
        # rospy.loginfo('Start Odom callback')

        self.loc = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.quat = msg.pose.pose.orientation
        if not self.odom_flag:
            self.odom_flag = 1

        #Publish curvature/steering angle and self.speed to "/drive"

    def create_segment(self, arr, N):
        '''
        inputs:
        -numpy array of
        '''
    #     a = arr[0]
    #     b = arr[1]
    #     np.linspace(a.position.x, b.position.x, N)
    #     np.linspace(a.position.y, b.position.y, N)
        x = []
        y = []
        for i in range(len(arr) - 2):
            a = arr[i]
            b = arr[i+1]
            x = x + np.linspace(a.position.x, b.position.x, N, endpoint=False).tolist()
            y = y + np.linspace(a.position.y, b.position.y, N, endpoint=False).tolist()
        a = arr[i+1]
        b = arr[i+2]
        x = x + np.linspace(a.position.x, b.position.x, N, endpoint=True).tolist()
        y = y + np.linspace(a.position.y, b.position.y, N, endpoint=True).tolist()

        segments = []
        for i in range(len(x) - 1):
            segments.append([[x[i], y[i]], [x[i+1], y[i+1]]])
        return np.array(segments)

    def find_closest_point(self, arr, loc):
        line_dx_array = arr[:,1,0] - arr[:,0,0] #line segment x deltas
        line_dy_array = arr[:,1,1] - arr[:,0,1] #line segment y deltas

        norms = np.square(line_dx_array) + np.square(line_dy_array)


        point_dx1 = loc[0] - arr[:,0,0] #x dist from point to start of segment
        point_dy1 = loc[1] - arr[:,0,1] #y dist from point to start of segment

        line_frac_array = (point_dx1 * line_dx_array + point_dy1 * line_dy_array) / norms #fractions along the line
        line_frac_array = np.clip(line_frac_array, a_min = 0, a_max = 1) #clipped so stays "on" the line segment

        x_array = arr[:,0,0] + line_frac_array * line_dx_array #x locations of closest points on line segments
        y_array = arr[:,0,1] + line_frac_array * line_dy_array #y locations of closest points on line segments


        dx_array = x_array - loc[0] #x dists to closest point on segment
        dy_array = y_array - loc[1] #y dists to closest point on segment

        dx2_array = np.square(dx_array) #squared x dists
        dy2_array = np.square(dy_array) #squared y dists

        dist_array = np.add(dx2_array, dy2_array) #array of dists from car location to closest point on all segments

        ix_min = np.argmin(dist_array) #get the index of the line segment the point is closest to

        #Might not need this
        #closest_point = np.array([x_array[min_dist_idx], y_array[min_dist_idx]]) #get the point car is closest to

        return ix_min


    def find_lookahead(self, arr, ix_min, loc, quat, L_dist, debug=False):
	# rospy.loginfo('ix_min')
	# rospy.loginfo(ix_min)


        for i in range(ix_min, arr.shape[0], 1):
            Q = loc                # Centre of circle is vehicle location
            r = L_dist             # Radius of circle is lookahead distance
            P1 = arr[i, 0, :]      # Start of line segment
            V = arr[i, 1, :] - P1  # Line segment vector (the endpoint if starts at origin)

            a = V.dot(V)
            b = 2 * V.dot(P1 - Q)
            c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r**2


            disc = b**2 - 4 * a * c
            if disc < 0:
                continue #circle doesn't intersect this line segment, try next one

	    #Fractions along the line segment where circle intersects
            t1 = (-b + np.sqrt(disc)) / (2.0 * a)
            t2 = (-b - np.sqrt(disc)) / (2.0 * a)

            # rospy.loginfo("t1: " + str(t1) + " t2: " + str(t2) + "\n" )
            if ((t1 < 1) & (t1 > 0)): #1st possible global frame intersection location

                lookahead_global = P1 + t1*V
                break

            elif ((t2 < 1) & (t2 > 0)): #2nd possible global frame intersection location
                lookahead_global = P1 + t2*V
                break

        lookahead_local = lookahead_global - loc

        roll, pitch, yaw = tf.transformations.euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
        yaw = yaw - np.pi/2
        R = np.array([[np.cos(yaw), np.sin(yaw)], \
                [-np.sin(yaw), np.cos(yaw)]])
        # rospy.loginfo(R)
        # rospy.loginfo(lookahead_local)
        # self.odom_sub.unregister()
        # rospy.loginfo("here")
        r_track = (L_dist**2)/np.dot(R, lookahead_local)[0]
        # rospy.loginfo("here")
        curvature = np.arctan(self.wheelbase_length/r_track)
        # rospy.loginfo(curvature)
        return curvature





if __name__=="__main__":
    rospy.init_node("pursuit_avoid")
    pf = PursuitAvoid()
    rospy.spin()
