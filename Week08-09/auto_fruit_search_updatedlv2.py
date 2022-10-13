# M4 - Autonomous fruit searching

# basic python packages
import sys, os
from tracemalloc import start
import cv2
import numpy as np
import json
import argparse
import time

# import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco

# import utility functions
sys.path.insert(0, "util")
from pibot import PenguinPi
import measure as measure

############## Kien: Packages for RRT algorithm #######
from Practical03_Support.Obstacle import *
import os

import random

# Import dependencies and set random seed
seed_value = 5
# 1. Set `PYTHONHASHSEED` environment variable at a fixed value
os.environ['PYTHONHASHSEED'] = str(seed_value)
# 2. Set `python` built-in pseudo-random generator at a fixed value
random.seed(seed_value)
# 3. Set `numpy` pseudo-random generator at a fixed value
np.random.seed(seed_value)
#######################################################

def read_true_map(fname):
    """Read the ground truth map and output the pose of the ArUco markers and 3 types of target fruit to search
    @param fname: filename of the map
    @return:
        1) list of target fruits, e.g. ['apple', 'pear', 'lemon']
        2) locations of the target fruits, [[x1, y1], ..... [xn, yn]]
        3) locations of ArUco markers in order, i.e. pos[9, :] = position of the aruco10_0 marker
    """
    with open(fname, 'r') as fd:
        gt_dict = json.load(fd)
        fruit_list = []
        fruit_true_pos = []
        aruco_true_pos = np.empty([10, 2])

        # remove unique id of targets of the same type
        for key in gt_dict:
            x = np.round(gt_dict[key]['x'], 1)
            y = np.round(gt_dict[key]['y'], 1)

            if key.startswith('aruco'):
                if key.startswith('aruco10'):
                    aruco_true_pos[9][0] = x
                    aruco_true_pos[9][1] = y
                else:
                    marker_id = int(key[5])
                    aruco_true_pos[marker_id-1][0] = x
                    aruco_true_pos[marker_id-1][1] = y
            else:
                fruit_list.append(key[:-2])
                if len(fruit_true_pos) == 0:
                    fruit_true_pos = np.array([[x, y]])
                else:
                    fruit_true_pos = np.append(fruit_true_pos, [[x, y]], axis=0)

        return fruit_list, fruit_true_pos, aruco_true_pos


def read_search_list():
    """Read the search order of the target fruits

    @return: search order of the target fruits
    """
    search_list = []
    with open('search_list.txt', 'r') as fd:
        fruits = fd.readlines()

        for fruit in fruits:
            search_list.append(fruit.strip())

    return search_list


def print_target_fruits_pos(search_list, fruit_list, fruit_true_pos):
    """Print out the target fruits' pos in the search order

    @param search_list: search order of the fruits
    @param fruit_list: list of target fruits
    @param fruit_true_pos: positions of the target fruits
    """

    print("Search order:")
    n_fruit = 1
    for fruit in search_list:
        for i in range(3):
            if fruit == fruit_list[i]:
                print('{}) {} at [{}, {}]'.format(n_fruit,
                                                  fruit,
                                                  np.round(fruit_true_pos[i][0], 1),
                                                  np.round(fruit_true_pos[i][1], 1)))
        n_fruit += 1


# Waypoint navigation
# the robot automatically drives to a given [x,y] coordinate
# additional improvements:
# you may use different motion model parameters for robot driving on its own or driving while pushing a fruit
# try changing to a fully automatic delivery approach: develop a path-finding algorithm that produces the waypoints

def drive_to_point(waypoint, robot_pose):
    # imports camera / wheel calibration parameters 
    fileS = "calibration/param/scale.txt"
    scale = np.loadtxt(fileS, delimiter=',')
    fileB = "calibration/param/baseline.txt"
    baseline = np.loadtxt(fileB, delimiter=',')
    
    ####################################################
    # TODO: replace with your codes to make the robot drive to the waypoint
    # One simple strategy is to first turn on the spot facing the waypoint,
    # then drive straight to the way point

    ## get robot pose/orientation and waypoint coords
    wp_x, wp_y = waypoint
    r_x, r_y, r_theta = robot_pose

    ## find distance and drive and angle to turn
    distance_to_drive = np.linalg.norm(np.array([wp_x,wp_y])-np.array([r_x,r_y]))
    angle_to_turn = np.arctan2(wp_y-r_y,wp_x-r_x) - r_theta
    if angle_to_turn < 0:   # account for negative angles
        angle_to_turn = 2*np.pi + angle_to_turn

    ## calculate time and turn and drive
    wheel_vel = 20 # ticks/sec
    scale = 4.537735840256848681e-3
    baseline = 1.720189921281630729e-1
    
    # turn to face waypoint
    turn_time = baseline*angle_to_turn/(scale*2*wheel_vel)
    print("Turning for {:.2f} seconds".format(turn_time))
    ppi.set_velocity([0, 1], turning_tick=wheel_vel, time=turn_time)
    
    # after turning, drive straight to the waypoint
    drive_time = distance_to_drive/(scale*wheel_vel)
    print("Driving for {:.2f} seconds".format(drive_time))
    ppi.set_velocity([1, 0], tick=wheel_vel, time=drive_time)
    ####################################################

    print("Arrived at [{}, {}]".format(waypoint[0], waypoint[1]))


def get_robot_pose(robot_pose, waypoint):
    ####################################################
    # TODO: replace with your codes to estimate the pose of the robot
    # We STRONGLY RECOMMEND you to use your SLAM code from M2 here

    # update the robot pose [x,y,theta]
    wp_x, wp_y = waypoint
    r_x,r_y,r_theta = robot_pose

    if waypoint == [0.0,0.0,0.0]:
        robot_pose = [0.0,0.0,0.0] # replace with your calculation
    else:
        x = wp_x
        y = wp_y
        theta = np.arctan2((wp_y-r_y),(wp_x-r_x))
    robot_pose = [x, y, theta]
    return robot_pose
    ####################################################

    return robot_pose

# putting slam in
def init_ekf(datadir, ip):
    fileK = "{}intrinsic.txt".format(datadir)
    camera_matrix = np.loadtxt(fileK, delimiter=',')
    fileD = "{}distCoeffs.txt".format(datadir)
    dist_coeffs = np.loadtxt(fileD, delimiter=',')
    fileS = "{}scale.txt".format(datadir)
    scale = np.loadtxt(fileS, delimiter=',')
    if ip == 'localhost':
        scale /= 2
    fileB = "{}baseline.txt".format(datadir)  
    baseline = np.loadtxt(fileB, delimiter=',')
    robot = Robot(baseline, scale, camera_matrix, dist_coeffs)
    return EKF(robot)


def update_slam(drive_meas):
    #img = take_pic(ppi) # be able to take an image
    #print(img)
    #lms, aruco_img = aruco_det.detect_marker_positions(img)
    #ekf.add_landmarks(lms)
    lms, self.aruco_img = self.aruco_det.detect_marker_positions(self.img)
    if self.request_recover_robot:
        is_success = self.ekf.recover_from_pause(lms)
        if is_success:
            self.notification = 'Robot pose is successfuly recovered'
            self.ekf_on = True
        else:
            self.notification = 'Recover failed, need >2 landmarks!'
            self.ekf_on = False
        self.request_recover_robot = False
    elif self.ekf_on: # and not self.debug_flag:
        self.ekf.predict(drive_meas)
        self.ekf.add_landmarks(lms)
        self.ekf.update(lms)

def take_pic(ppi):
    img = ppi.get_image()
    return img

################### KIEN: RRT ALGORITHM ############################
# This is an adapted version of the RRT implementation done by Atsushi Sakai (@Atsushi_twi)
class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, start=np.zeros(2),
                 goal=np.array([120,90]),
                 obstacle_list=None,
                 width = 160,
                 height=100,
                 expand_dis=3.0, 
                 path_resolution=0.5, 
                 max_points=200):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacle_list: list of obstacle objects
        width, height: search area
        expand_dis: min distance between random node and closest node in rrt to it
        path_resolion: step size to considered when looking for node to expand
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.width = width
        self.height = height
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.max_nodes = max_points
        self.obstacle_list = obstacle_list
        self.node_list = []

    def planning(self, animation=True):
        """
        rrt path planning
        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        while len(self.node_list) <= self.max_nodes:
            
            # 1. Generate a random node           
            rnd_node = self.get_random_node()
            
            # 2. Find node in tree that is closest to sampled node.
            # This is the node to be expanded (q_expansion)
            expansion_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            expansion_node = self.node_list[expansion_ind]

            #TODO:  Complete the last two main steps of the RRT algorithm ----------------
            # 3. Select a node (nearby_node) close to expansion_node by moving from expantion_node to rnd_node
            # Use the steer method
            nearby_node = self.steer(expansion_node, rnd_node, self.expand_dis)            
            # 4. Check if nearby_node is in free space (i.e., it is collision free). If collision free, add node
            # to self.node_list
            if self.is_collision_free(nearby_node):
                self.node_list.append(nearby_node)            
            # Please remove return None when you start coding
            #ENDTODO -----------------------------------------------------------------------
                
            # If we are close to goal, stop expansion and generate path
            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.is_collision_free(final_node):
                    return self.generate_final_course(len(self.node_list) - 1)

        return None  # cannot find path

    
    def steer(self, from_node, to_node, extend_length=float("inf")):
        """
        Given two nodes from_node, to_node, this method returns a node new_node such that new_node 
        is “closer” to to_node than from_node is.
        """
        
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        cos_theta, sin_theta = np.cos(theta), np.sin(theta)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        # How many intermediate positions are considered between from_node and to_node
        n_expand = math.floor(extend_length / self.path_resolution)

        # Compute all intermediate positions
        for _ in range(n_expand):
            new_node.x += self.path_resolution * cos_theta
            new_node.y += self.path_resolution * sin_theta
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

        new_node.parent = from_node

        return new_node


    def is_collision_free(self, new_node):
        """
        Determine if nearby_node (new_node) is in the collision-free space.
        """
        if new_node is None:
            return True

        points = np.vstack((new_node.path_x, new_node.path_y)).T
        for obs in self.obstacle_list:
            in_collision = obs.is_in_collision_with_points(points)
            if in_collision:
                return False
        
        return True  # safe
        
    
    def generate_final_course(self, goal_ind):
        """
        Reconstruct path from start to end node
        """
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        x = self.width * np.random.random_sample()
        y = self.height * np.random.random_sample()
        rnd = self.Node(x, y)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        # Compute Euclidean disteance between rnd_node and all nodes in tree
        # Return index of closest element
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy) #returns the Euclidean norm
        theta = math.atan2(dy, dx)
        return d, theta    
#######################################################################
    
# main loop
if __name__ == "__main__":
    parser = argparse.ArgumentParser("Fruit searching")
    parser.add_argument("--map", type=str, default='M4_true_map.txt')
    parser.add_argument("--ip", metavar='', type=str, default='localhost')
    parser.add_argument("--port", metavar='', type=int, default=40000)
    parser.add_argument("--calib_dir", type=str, default="calibration/param/")
    args, _ = parser.parse_known_args()

    ppi = PenguinPi(args.ip,args.port)

    # read in the true map
    fruits_list, fruits_true_pos, aruco_true_pos = read_true_map(args.map)
    search_list = read_search_list()
    print_target_fruits_pos(search_list, fruits_list, fruits_true_pos)

    # start up SLAM
    ekf = init_ekf(args.calib_dir, args.ip) # initialise EKF
    aruco_det = aruco.aruco_detector(ekf.robot, marker_length = 0.07) # initialise aruco_detector

    # load true aruco marker positions into ekf
    for lm in aruco_true_pos:
        ekf.markers = np.concatenate((ekf.markers, np.array([[lm[0]],[lm[1]]])), axis=1)
        ekf.P = np.concatenate((ekf.P, np.zeros((2, ekf.P.shape[1]))), axis=0)
        ekf.P = np.concatenate((ekf.P, np.zeros((ekf.P.shape[0], 2))), axis=1)
        ekf.P[-2,-2] = ekf.init_lm_cov**2
        ekf.P[-1,-1] = ekf.init_lm_cov**2
    ekf.taglist = list(range(1,11))
    #print(ekf.markers)
    #print(ekf.taglist)

    waypoint = [0.0,0.0]
    robot_pose = [0.0,0.0,0.0]

################### KIEN: RRT ALGORITHM ###################
    goal_i = fruits_true_pos[0]    # fruit pos[j] -> need loop: Replace -> next fruit pos[j+1]
    start_i = np.array([0.0, 0.0]) # start -> Replace -> fruit pos[j]
    all_obstacles = []
    # Take obstacles from the map
    for i in range(len(aruco_true_pos)):
        all_obstacles.append(Circle(aruco_true_pos[i,0],aruco_true_pos[i,1],0.03)) # Make circle around obstacle (assume r = 0.03) to avoid

    for j in range(len(fruits_true_pos)):  # Loop to generate paths for many fruits
    ## RRT generate path to 1 fruit -> need generate multiple RRT for different fruits pos ##                     
        #### Adjust path searching area (width & height) ####
        if goal_i[0] < start_i[0]:
            width_i = -5
        else:
            width_i = 5
        if goal_i[1] < start_i[1]:
            height_i = -5
        else:
            height_i = 5
        rrt = RRT(start=start_i, goal=goal_i, width = width_i, height = height_i, obstacle_list=all_obstacles, expand_dis=1, path_resolution=0.5)
        #################################################
        path = rrt.planning()
    # The following code is only a skeleton code the semi-auto fruit searching task
        while True:
            # enter the waypoints
            # instead of manually enter waypoints, you can get coordinates by clicking on a map, see camera_calibration.py

    #### KIEN: Change from input waypoint --> take waypoint from generated path: ######
            for k in range(len(path)-1,-1,-1):
         #### Adjust goal position (not fruit position) to avoid hitting fruit ####
                if k == 0:
                    vector[0] = path[k][0] - path[k+1][0]
                    vector[1] = path[k][1] - path[k+1][1]
                    if vector[0] > 0:                 # If vector = 0 -> No change
                        path[k][0] = path[k][0] - 0.02
                    if vector[0] < 0:
                        path[k][0] = path[k][0] + 0.02
                    if vector[1] > 0:
                        path[k][1] = path[k][1] - 0.02
                    if vector[1] < 0:
                        path[k][1] = path[k][1] + 0.02             
                x = path[k][0]
                y = path[k][1]
                # robot drives to the waypoint
                waypoint = [x,y]
                drive_to_point(waypoint, robot_pose)
                # update robot pose
                robot_pose = get_robot_pose(robot_pose, waypoint) # update robot pose
                img = take_pic(ppi) # use camera
                lms, aruco_img = aruco_det.detect_marker_positions(img)
                drive_meas = measure.Drive(1,1,1,10000,10000)
                ekf.predict(drive_meas)
                #print(lms)
                ekf.update(lms)
                #print(ekf.get_state_vector())
                #print("Finished driving to waypoint: {}; New robot pose: {}".format(waypoint,robot_pose))
                # exit
                ppi.set_velocity([0, 0])
                #uInput = input("Add a new waypoint? [Y/N]")
                #if uInput == 'N' or uInput == 'n':
                #    break
        if j == len(fruits_true_pos)-1:
            break
        time.sleep(3)
        start_i[0] = path[k][0]   # Set Start: Current position ( previous Goal)
        start_i[1] = path[k][1]
        goal_i = fruits_true_pos[j+1]  # Set next Goal: Next fruit position
