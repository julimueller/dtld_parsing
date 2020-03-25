from __future__ import print_function

__author__ = "Andreas Fregin, Julian Mueller and Klaus Dietmayer"
__maintainer__ = "Julian Mueller"
__email__ = "julian.mu.mueller@daimler.com"

import cv2
import yaml
import numpy as np
np.set_printoptions(suppress=True)
import os.path
import copy
import time
import argparse
import logging
import sys

from calibration import (
  IntrinsicCalibration,
  ExtrinsicCalibration,
  RectificationMatrix,
  ProjectionMatrix,
  DistortionCalibration,
  CalibrationData
)

from driveu_dataset import DriveuDatabase

# Logging
logging.basicConfig(
    stream=sys.stdout,
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d %(levelname)s %(module)s - %(funcName)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)


def parse_args():

  parser = argparse.ArgumentParser()
  parser.add_argument('--label_file', default='')
  parser.add_argument('--calib_dir', default='')
  parser.add_argument('--data_base_dir', default='')
  return parser.parse_args()


def main(args):

    # Load database
    database = DriveuDatabase(args.label_file)
    database.open(args.data_base_dir)

    # Load calibration
    calibration = CalibrationData()
    intrinsic_left = calibration.load_intrinsic_matrix(args.calib_dir + '/intrinsic_left.yml')
    rectification_left = calibration.load_rectification_matrix(args.calib_dir + '/rectification_left.yml')
    projection_left = calibration.load_projection_matrix(args.calib_dir + '/projection_left.yml')
    extrinsic = calibration.load_extrinsic_matrix(args.calib_dir + '/extrinsic.yml')
    distortion_left = calibration.load_distortion_matrix(args.calib_dir + '/distortion_left.yml')

    logging.info("Intrinsic Matrix:\n\n{}\n".format(intrinsic_left))
    logging.info("Extrinsic Matrix:\n\n{}\n".format(extrinsic))
    logging.info("Projection Matrix:\n\n{}\n".format(projection_left))
    logging.info("Rectification Matrix:\n\n{}\n".format(rectification_left))
    logging.info("Distortion Matrix:\n\n{}\n".format(distortion_left))

    # Visualize image by image
    for idx_d, img in enumerate(database.images):
        # Get disparity image
        img_disp = img.visualize_disparity_image()
        rects = img.map_labels_to_disparity_image(calibration)
        # Plot labels into disparity image
        for rect in rects:
            cv2.rectangle(img_disp, (int(rect[0]), int(rect[1])), (int(rect[0]) + int(rect[2]), int(rect[1]) + int(rect[3])), (255,255,255), 2)
        # Get color image with labels
        img_color = img.get_labeled_image()
        img_color = cv2.resize(img_color, (1024, 440))
        # Plot side by side
        img_concat = np.concatenate((img_color, img_disp), axis=1)
        cv2.imshow("DTLD_visualized", img_concat)
        cv2.waitKey(1)

if __name__ == '__main__':
  main(parse_args())


