from __future__ import print_function

import argparse
import logging
import sys

import matplotlib.pyplot as plt
import numpy as np

import cv2
from dtld_parsing.calibration import CalibrationData
from dtld_parsing.driveu_dataset import DriveuDatabase

__author__ = "Andreas Fregin, Julian Mueller and Klaus Dietmayer"
__maintainer__ = "Julian Mueller"
__email__ = "julian.mu.mueller@daimler.com"


np.set_printoptions(suppress=True)


# Logging
logging.basicConfig(
    stream=sys.stdout,
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d %(levelname)s %(module)s - %(funcName)s: "
    "%(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)


def parse_args():

    parser = argparse.ArgumentParser()
    parser.add_argument("--label_file",
                        help="DTLD label files (.json)",
                        type=str,
                        required=True)
    parser.add_argument("--calib_dir",
                        help="calibration directory where .yml are stored",
                        type=str,
                        required=True)
    parser.add_argument("--data_base_dir",
                        default="",
                        help="only use this if the image file paths in the"
                        "label files are not up to date. Do NOT change the"
                        "internal DTLD folder structure!",
                        type=str)
    return parser.parse_args()


def main(args):

    # Load database
    database = DriveuDatabase(args.label_file)
    if not database.open(args.data_base_dir):
        return False

    # Load calibration
    calibration = CalibrationData()
    intrinsic_left = calibration.load_intrinsic_matrix(
        args.calib_dir + "/intrinsic_left.yml"
    )
    rectification_left = calibration.load_rectification_matrix(
        args.calib_dir + "/rectification_left.yml"
    )
    projection_left = calibration.load_projection_matrix(
        args.calib_dir + "/projection_left.yml"
    )
    extrinsic = calibration.load_extrinsic_matrix(
        args.calib_dir + "/extrinsic.yml"
    )
    distortion_left = calibration.load_distortion_matrix(
        args.calib_dir + "/distortion_left.yml"
    )

    logging.info("Intrinsic Matrix:\n\n{}\n".format(intrinsic_left))
    logging.info("Extrinsic Matrix:\n\n{}\n".format(extrinsic))
    logging.info("Projection Matrix:\n\n{}\n".format(projection_left))
    logging.info("Rectification Matrix:\n\n{}\n".format(rectification_left))
    logging.info("Distortion Matrix:\n\n{}\n".format(distortion_left))

    # create axes
    ax1 = plt.subplot(111)

    # Visualize image by image
    for idx_d, img in enumerate(database.images):
        # Get disparity image
        img_disp = img.visualize_disparity_image()
        rects = img.map_labels_to_disparity_image(calibration)
        # Plot labels into disparity image
        for rect in rects:
            cv2.rectangle(
                img_disp,
                (int(rect[0]), int(rect[1])),
                (int(rect[0]) + int(rect[2]), int(rect[1]) + int(rect[3])),
                (255, 255, 255),
                2,
            )
        # Get color image with labels
        img_color = img.get_labeled_image()
        img_color = cv2.resize(img_color, (1024, 440))
        # Plot side by side
        img_concat = np.concatenate((img_color, img_disp), axis=1)
        # Because of the weird qt error in gui methods in opencv-python >= 3
        # imshow does not work in some cases. You can try it by yourself.
        # cv2.imshow("DTLD_visualized", img_concat)
        # cv2.waitKey(1)
        img_concat_rgb = img_concat[..., ::-1]
        if idx_d == 0:
            im1 = ax1.imshow(img_concat_rgb)
        plt.ion()
        im1.set_data(img_concat_rgb)
        plt.pause(0.001)
        plt.draw()


if __name__ == "__main__":
    main(parse_args())
