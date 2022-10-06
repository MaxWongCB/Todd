import numpy as np
import cv2 
import os, sys
import time
import matplotlib.pyplot as plt
import matplotlib.patches as label_box

# import utility functions
sys.path.insert(0, "{}/utility".format(os.getcwd()))
from util.pibot import PenguinPi # access the robot
import util.DatasetHandler as dh # save/load functions
import util.measure as measure # measurements
import pygame # python package for GUI
import shutil # python package for file operations

# import SLAM components you developed in M2
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco

# import CV components
sys.path.insert(0,"{}/network/".format(os.getcwd()))
sys.path.insert(0,"{}/network/scripts".format(os.getcwd()))
from network.scripts.detector import Detector

ckpt = 'network/scripts/model/model.best.pth'
detector = Detector(ckpt, use_gpu=False)

test_dir = "~/ECE4078_Lab_2022/"
pred_dir = test_dir#os.path.join(test_dir, "pred")
#os.makedirs(pred_dir, exist_ok=True)
all_test_images = [file for file in os.listdir(test_dir) if file.endswith('.png')] 
for image_name in all_test_images:
    np_img = np.array(Image.open(os.path.join(test_dir, image_name)))
    pred, colour_map = detector.detect_single_image(np_img)
    title = ["Input", "Prediction"]
    pics = [np_img, colour_map]
    fig, axs = plt.subplots(1, 2, figsize=(15, 10))
    axs[0].imshow(pics[0], interpolation='nearest')
    axs[0].set_title(title[0])
    axs[1].imshow(pics[1], interpolation='nearest')
    axs[1].set_title(title[1])
    axs[0].axis('off')
    axs[1].axis('off')
    path = os.path.join(pred_dir, image_name)
    plt.savefig(os.path.join(pred_dir, image_name[:-4]+'.jpg'))