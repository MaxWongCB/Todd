# M4 - Autonomous fruit searching

# basic python packages
import sys, os
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

def drive_to_point(waypoint, robot_pose, ekf):
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
    angle_to_face = np.arctan2(wp_y-r_y,wp_x-r_x)
    angle_to_turn = angle_to_face - r_theta
    #if angle_to_turn < 0:   # account for negative angles
    #    angle_to_turn = 2*np.pi + angle_to_turn

    ## calculate time and turn and drive
    wheel_vel = 10 # ticks/sec
    scale = 4.537735840256848681e-3
    baseline = 1.720189921281630729e-1
    
    # turn to face waypoint
    dt = 0.05
    turn_time = baseline*abs(angle_to_turn)/(scale*2*wheel_vel)
    print("Turning for {:.2f} seconds".format(turn_time))
    # keep turning until either timer runs out or it reaches right angle
    count = 0
    while ((ekf.robot.state[2][0] > angle_to_face+0.05) or (ekf.robot.state[2][0] < angle_to_face-0.05)) and (count < turn_time/dt):
        if angle_to_turn < 0:
            lv, rv = ppi.set_velocity([0, -1], turning_tick=wheel_vel, time=dt)
        else:
            lv, rv = ppi.set_velocity([0, 1], turning_tick=wheel_vel, time=dt)
        drive_meas = measure.Drive(lv,rv,2*dt,100000,100000)   # should change it based on simulation time?
        #print(ekf.robot.state)
        ekf.predict(drive_meas)
        ekf.robot.state = clip_angle(ekf.robot.state)
        #print(ekf.robot.state)
        img = take_pic(ppi) # use camera
        lms, aruco_img = aruco_det.detect_marker_positions(img)
        #print("lms: ",lms)
        ekf.update(lms)
        ekf.robot.state = clip_angle(ekf.robot.state)
        #print(ekf.robot.state)
        #print(ekf.get_state_vector())
        count += 1
    

    # after turning, drive straight to the waypoint
    ## get robot pose/orientation and waypoint coords
    r_x, r_y, r_theta = get_robot_pose(ekf)
    wheel_vel = 20 # ticks/sec
    ## find distance and drive and angle to turn
    distance_to_drive = np.linalg.norm(np.array([wp_x,wp_y])-np.array([r_x,r_y]))
    drive_time = distance_to_drive/(scale*wheel_vel)
    
    print("Driving for {:.2f} seconds".format(drive_time))
    #dt = 1
    #count = 0
    #while (count < drive_time/dt):
    lv, rv = ppi.set_velocity([1, 0], tick=wheel_vel, time=drive_time)
    drive_meas = measure.Drive(lv,rv,2*drive_time,100,100)
    ekf.predict(drive_meas)
    ekf.robot.state = clip_angle(ekf.robot.state)
    #print(ekf.robot.state)
    img = take_pic(ppi) # use camera
    lms, aruco_img = aruco_det.detect_marker_positions(img)
    #print("lms: ",lms)
    ekf.update(lms)
    ekf.robot.state = clip_angle(ekf.robot.state)
    #count += 1
    #print(ekf.robot.state)
        ####################################################

    print("Arrived at [{}, {}]".format(waypoint[0], waypoint[1]))
    return ekf

def clip_angle(ekf_state):
    angle = ekf_state[2][0]
    if angle > np.pi:
        angle = angle - 2*np.pi
    ekf_state[2][0] = angle
    return ekf_state


def get_robot_pose(ekf):
    ####################################################
    # TODO: replace with your codes to estimate the pose of the robot
    # We STRONGLY RECOMMEND you to use your SLAM code from M2 here

    # update the robot pose [x,y,theta]
    '''
    wp_x, wp_y = waypoint
    r_x,r_y,r_theta = robot_pose

    if waypoint == [0.0,0.0,0.0]:
        robot_pose = [0.0,0.0,0.0] # replace with your calculation
    else:
        x = wp_x
        y = wp_y
        theta = np.arctan2((wp_y-r_y),(wp_x-r_x))
    robot_pose = [x, y, theta]
    '''

    # new version
    robot_pose[0] = ekf.robot.state[0][0]
    robot_pose[1] = ekf.robot.state[1][0]
    robot_pose[2] = ekf.robot.state[2][0]
    if robot_pose[2] > np.pi:
        robot_pose[2] = robot_pose[2] - 2*np.pi
    return robot_pose
    ####################################################

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

def take_pic(ppi):
    img = ppi.get_image()
    return img

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
        #ekf.P = np.concatenate((ekf.P, np.zeros((2, ekf.P.shape[1]))), axis=0)
        #ekf.P = np.concatenate((ekf.P, np.zeros((ekf.P.shape[0], 2))), axis=1)
        #ekf.P[-2,-2] = ekf.init_lm_cov**2
        #ekf.P[-1,-1] = ekf.init_lm_cov**2
    ekf.taglist = list(range(1,11))

    waypoint = [0.0,0.0]
    robot_pose = [0.0,0.0,0.0]

    # The following code is only a skeleton code the semi-auto fruit searching task
    while True:
        # enter the waypoints
        # instead of manually enter waypoints, you can get coordinates by clicking on a map, see camera_calibration.py
        x,y = 0.0,0.0
        x = input("X coordinate of the waypoint: ")
        try:
            x = float(x)
        except ValueError:
            print("Please enter a number.")
            continue
        y = input("Y coordinate of the waypoint: ")
        try:
            y = float(y)
        except ValueError:
            print("Please enter a number.")
            continue

        # robot drives to the waypoint
        waypoint = [x,y]

        ekf = drive_to_point(waypoint, robot_pose, ekf)
        #robot_pose = ekf.robot.state
        # update robot pose
        #print(ekf.robot.state)
        robot_pose = get_robot_pose(ekf) # update robot pose
        #img = take_pic(ppi) # use camera
        #lms, aruco_img = aruco_det.detect_marker_positions(img)
        #drive_meas = measure.Drive(1,1,1,10000,10000)
        #ekf.predict(drive_meas)
        #print(lms)
        #ekf.update(lms)
        #print(ekf.get_state_vector())

        print("Finished driving to waypoint: {}; New robot pose: {}".format(waypoint,robot_pose))

        # exit
        ppi.set_velocity([0, 0])
        uInput = input("Add a new waypoint? [Y/N]")
        if uInput == 'N' or uInput == 'n':
            break