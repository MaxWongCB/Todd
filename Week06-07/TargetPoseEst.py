# estimate the pose of a target object detected
import numpy as np
import json
import os, sys
from pathlib import Path
import ast
# import cv2
import math
from machinevisiontoolbox import Image

import matplotlib.pyplot as plt
import PIL

sys.path.insert(0,"{}/network/".format(os.getcwd()))
sys.path.insert(0,"{}/network/scripts".format(os.getcwd()))
from network.scripts.detector import Detector

def detect_target(img):
    # Takes an image and detects the targets within it
    ckpt = 'network/scripts/model/model.best.pth'
    detector = Detector(ckpt, use_gpu=False)
    detector_output, network_vis = detector.detect_single_image(img)
    #self.command['inference'] = False
    #self.file_output = (self.detector_output, self.ekf)
    #print(detector_output)# = f'{len(np.unique(self.detector_output))-1} target type(s) detected'
    return detector_output

# use the machinevision toolbox to get the bounding box of the detected target(s) in an image
def get_bounding_box(target_number, image_path):
    image = PIL.Image.open(image_path).resize((640,480), PIL.Image.NEAREST)
    target = Image(image)==target_number
    blobs = target.blobs()
    [[u1,u2],[v1,v2]] = blobs[0].bbox # bounding box
    width = abs(u1-u2)
    height = abs(v1-v2)
    center = np.array(blobs[0].centroid).reshape(2,)
    box = [center[0], center[1], int(width), int(height)] # box=[x,y,width,height]
    # plt.imshow(fruit.image)
    # plt.annotate(str(fruit_number), np.array(blobs[0].centroid).reshape(2,))
    # plt.show()
    # assert len(blobs) == 1, "An image should contain only one object of each target type"
    return box

# read in the list of detection results with bounding boxes and their matching robot pose info
def get_image_info(base_dir, file_path, image_poses):
    # there are at most five types of targets in each image
    target_lst_box = [[], [], [], [], []]
    target_lst_pose = [[], [], [], [], []]
    completed_img_dict = {}

    # add the bounding box info of each target in each image
    # target labels: 1 = apple, 2 = lemon, 3 = pear, 4 = orange, 5 = strawberry, 0 = not_a_target
    
    #np_img = np.array(Image.open(os.path.join(test_dir, image_name)))
    #pred, colour_map = detector.detect_single_image(np_img)
    seg_img = detect_target(Image(base_dir / file_path).image)
    img_vals = set(seg_img.reshape(-1))
    for target_num in img_vals:
        if target_num > 0:
            try:
                box = get_bounding_box(target_num, base_dir/file_path) # [x,y,width,height]
                pose = image_poses[file_path] # [x, y, theta]
                target_lst_box[target_num-1].append(box) # bouncing box of target
                target_lst_pose[target_num-1].append(np.array(pose).reshape(3,)) # robot pose
            except ZeroDivisionError:
                pass

    # if there are more than one objects of the same type, combine them
    for i in range(5):
        if len(target_lst_box[i])>0:
            box = np.stack(target_lst_box[i], axis=1)
            pose = np.stack(target_lst_pose[i], axis=1)
            completed_img_dict[i+1] = {'target': box, 'robot': pose}
        
    return completed_img_dict

# estimate the pose of a target based on size and location of its bounding box in the robot's camera view and the robot's pose
def estimate_pose(base_dir, camera_matrix, completed_img_dict):
    camera_matrix = camera_matrix
    focal_length = camera_matrix[0][0]
    # actual sizes of targets [For the simulation models]
    # You need to replace these values for the real world objects
    target_dimensions = []
    apple_dimensions = [0.075448, 0.074871, 0.071889]
    target_dimensions.append(apple_dimensions)
    lemon_dimensions = [0.060588, 0.059299, 0.053017]
    target_dimensions.append(lemon_dimensions)
    pear_dimensions = [0.0946, 0.0948, 0.135]
    target_dimensions.append(pear_dimensions)
    orange_dimensions = [0.0721, 0.0771, 0.0739]
    target_dimensions.append(orange_dimensions)
    strawberry_dimensions = [0.052, 0.0346, 0.0376]
    target_dimensions.append(strawberry_dimensions)

    target_list = ['apple', 'lemon', 'pear', 'orange', 'strawberry']

    target_pose_dict = {}
    # for each target in each detection output, estimate its pose
    for target_num in completed_img_dict.keys():
        box = completed_img_dict[target_num]['target'] # [[x],[y],[width],[height]]
        robot_pose = completed_img_dict[target_num]['robot'] # [[x], [y], [theta]]
        true_height = target_dimensions[target_num-1][2]
        
        ######### Replace with your codes #########
        # TODO: compute pose of the target based on bounding box info and robot's pose
        x_robot = focal_length*true_height/box[3]
        y_robot = -x_robot*box[0]/focal_length
        distance = np.sqrt(x_robot**2 + y_robot**2)
        phi = np.arctan(y_robot/x_robot)
        theta = robot_pose[2]
        y_world = robot_pose[1] + distance*np.sin(theta-phi)
        x_world = robot_pose[0] + distance*np.cos(theta-phi)
        target_pose = {'y': y_world, 'x': x_world}
        
        target_pose_dict[target_list[target_num-1]] = target_pose
        ###########################################
    
    return target_pose_dict

# merge the estimations of the targets so that there are at most 3 estimations of each target type
def merge_estimations(target_pose_dict):
    target_pose_dict = target_pose_dict
    apple_est, lemon_est, pear_est, orange_est, strawberry_est = [], [], [], [], []
    target_est = {}
    
    # combine the estimations from multiple detector outputs
    for f in target_map:
        for key in target_map[f]:
            if key.startswith('apple'):
                apple_est.append(np.array(list(target_map[f][key].values()), dtype=float))
            elif key.startswith('lemon'):
                lemon_est.append(np.array(list(target_map[f][key].values()), dtype=float))
            elif key.startswith('pear'):
                pear_est.append(np.array(list(target_map[f][key].values()), dtype=float))
            elif key.startswith('orange'):
                orange_est.append(np.array(list(target_map[f][key].values()), dtype=float))
            elif key.startswith('strawberry'):
                strawberry_est.append(np.array(list(target_map[f][key].values()), dtype=float))

    ######### Replace with your codes #########
    # TODO: the operation below takes the first two estimations of each target type, replace it with a better merge solution
    # I have implemented a very simple method by taking extreme of positions
    # Its better to use some sort of average, but this should give a good enough approximation for now
    if len(apple_est) > 2:
        apple_est = np.sort(apple_est, axis=0)
        apple_est = [apple_est[0], apple_est[-1]]
    if len(lemon_est) > 2:
        lemon_est = np.sort(lemon_est, axis=0)
        lemon_est = [lemon_est[0], lemon_est[-1]]
    if len(pear_est) > 2:
        pear_est = np.sort(pear_est, axis=0)
        pear_est = [pear_est[0], pear_est[-1]]
    if len(orange_est) > 2:
        orange_est = np.sort(orange_est, axis=0)
        orange_est = [orange_est[0], orange_est[-1]]
    if len(strawberry_est) > 2:
        strawberry_est = np.sort(strawberry_est, axis=0)
        strawberry_est = [strawberry_est[0], strawberry_est[-1]]

    for i in range(2):
        try:
            target_est['apple_'+str(i)] = {'y':apple_est[i][0][0], 'x':apple_est[i][1][0]}
        except:
            pass
        try:
            target_est['lemon_'+str(i)] = {'y':lemon_est[i][0][0], 'x':lemon_est[i][1][0]}
        except:
            pass
        try:
            target_est['pear_'+str(i)] = {'y':pear_est[i][0][0], 'x':pear_est[i][1][0]}
        except:
            pass
        try:
            target_est['orange_'+str(i)] = {'y':orange_est[i][0][0], 'x':orange_est[i][1][0]}
        except:
            pass
        try:
            target_est['strawberry_'+str(i)] = {'y':strawberry_est[i][0][0], 'x':strawberry_est[i][1][0]}
        except:
            pass
    ###########################################
        #print(target_est)
    return target_est



if __name__ == "__main__":
    # camera_matrix = np.ones((3,3))/2
    fileK = "{}intrinsic.txt".format('./calibration/param/')
    camera_matrix = np.loadtxt(fileK, delimiter=',')
    base_dir = Path('./')
    
    
    # a dictionary of all the saved detector outputs
    image_poses = {}
    with open(base_dir/'lab_output/images.txt') as fp:
        for line in fp.readlines():
            pose_dict = ast.literal_eval(line)
            image_poses[pose_dict['imgfname']] = pose_dict['pose']
    
    # estimate pose of targets in each detector output
    target_map = {}        
    for file_path in image_poses.keys():
        completed_img_dict = get_image_info(base_dir, file_path, image_poses)
        target_map[file_path] = estimate_pose(base_dir, camera_matrix, completed_img_dict)

    # merge the estimations of the targets so that there are at most 3 estimations of each target type
    target_est = merge_estimations(target_map)
                     
    # save target pose estimations
    with open(base_dir/'lab_output/targets.txt', 'w') as fo:
        json.dump(target_est, fo)
    
    print('Estimations saved!')



