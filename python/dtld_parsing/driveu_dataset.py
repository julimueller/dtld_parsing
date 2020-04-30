from __future__ import print_function

import copy
import json
import logging
import os
import sys

import numpy as np

import cv2
from dtld_parsing.calibration import CalibrationData
from dtld_parsing.vehicle_data import VehicleData

__author__ = "Andreas Fregin, Julian Mueller and Klaus Dietmayer"
__maintainer__ = "Julian Mueller"
__email__ = "julian.mu.mueller@daimler.com"

"""
DTLD_v1 suppports only .yml files
DTLD_v2(04/2020) only supports .json files
"""


class DriveuObject:
    """
    Class holding properties of a label object in the dataset

    Attributes:
        x(int):             X coordinate of upper left corner of bbbox label
        y(int):             Y coordinate of upper left corner of bbox label
        width(int):         Width of bounding box label
        height(int):        Height of bounding box label
        attributes(dict):   Attributes dictionary with label attributes
                            "direction" =   ["front" || "back" || "left"
                            || "right"],
                            "occlusion" = ["occluded" || "not_occluded"],
                            "relevance" = ["relevant" || "not_relevant"],
                            "orientation" = ["vertical" || "horizontal"],
                            "aspects" = ["one_aspect" || "two_aspects",
                            || "three_aspects" || "four_aspects" || "unknown"],
                            "state" = ["red" || "green" || "yellow"
                            || "red_yellow" || "off" || "unknown"]
                            "pictogram" = ["circle" || "arrow_left"
                            || "arrow_right" || "arrow_straight" || "tram"
                            || "pedestrian" || "bicycle" || unknown]
        unique_id(int):  Unique ID of the object
        track_id(string):Track ID of the objec (representing one real-world
        TL instance)
    """

    def __init__(self):
        self.x = 0
        self.y = 0
        self.width = 0
        self.height = 0
        self.attributes = {}
        self.unique_id = 0
        self.track_id = 0

    def parse_object_dict(self, object_dict: dict):
        """
        Method loading label data from json file dict

        Args:
            object_dict(dict): label dictionary read from json file
        """
        self.x = object_dict["x"]
        self.y = object_dict["y"]
        self.width = object_dict["w"]
        self.height = object_dict["h"]
        self.attributes = object_dict["attributes"]
        self.unique_id = object_dict["unique_id"]
        self.track_id = object_dict["track_id"]

    def color_from_attributes(self):
        """
        Return color for state of class identity

        Returns:
            Color-vector (BGR) for traffic light visualization
        """
        # Second last digit indicates state/color
        if self.attributes["state"] == "red":
            return (0, 0, 255)
        elif self.attributes["state"] == "yellow":
            return (0, 255, 255)
        elif self.attributes["state"] == "red_yellow":
            return (0, 165, 255)
        elif self.attributes["state"] == "green":
            return (0, 255, 0)
        else:
            return (255, 255, 255)


class DriveuImage:
    """
    Class holding properties of one image in the DriveU Database

    Attributes:
        file_path (string):         Path of the left camera image
        disp_file_path (string):    Path of the corresponding disparity image
        timestamp (float):          Timestamp of the image
        vehicle_data (VehicleData):  Vehicle Data at that timestamp
        objects (DriveuObject)     : Labels in that image
    """

    def __init__(self):
        self.file_path = ""
        self.disp_file_path = ""
        self.timestamp = 0
        self.vehicle_data = VehicleData()
        self.objects = []

    def parse_image_dict(self, image_dict: dict, data_base_dir: str = ""):
        """
        Method loading image data from json file dict

        Args:
            image_dict(dict): image dictionary read from json label file
            data_base_dir(str): optional, if file paths are oudated
            (DTLD was moved from directory). Note that the internal
            DTLD should not be changed!
        """
        # Parse images
        if data_base_dir != "":
            inds = [i for i, c in enumerate(image_dict["image_path"]) if c == "/"]
            self.file_path = os.path.join(data_base_dir,
                                          image_dict["image_path"][inds[-4]:].strip("/"))
            inds = [
                i for i, c in enumerate(image_dict["disparity_image_path"]) if c == "/"
            ]
            self.disp_file_path = os.path.join(data_base_dir,
                                        image_dict["disparity_image_path"][inds[-4]:].strip("/"))

        else:
            self.file_path = image_dict["image_path"]
            self.disp_file_path = image_dict["disparity_image_path"]
            self.timestamp = image_dict["time_stamp"]

        # Parse vehicle data
        self.vehicle_data.parse_vehicle_data_dict(image_dict)

        # Parse labels
        for o in image_dict["labels"]:
            label = DriveuObject()
            label.parse_object_dict(o)
            self.objects.append(label)

    def get_image(self):
        """
        Method loading the left unrectified color image in 8 bit

        Returns:
            (bool np.array): (status, 8 Bit BGR color image)
        """
        if os.path.isfile(self.file_path):
            # Load image from file path, do debayering and shift
            img = cv2.imread(self.file_path, cv2.IMREAD_UNCHANGED)
            if os.path.splitext(self.file_path)[1] == ".tiff":
                img = cv2.cvtColor(img, cv2.COLOR_BAYER_GB2BGR)
                # Images are saved in 12 bit raw -> shift 4 bits
                img = np.right_shift(img, 4)
                img = img.astype(np.uint8)

            return True, img

        else:
            logging.error("Image {} not found. Please check image file paths!".format(self.file_path))
            sys.exit(1)
            return False, np.array()

    def get_labeled_image(self):
        """
        Method loading the left unrectified color image with visualized labels

        Returns:
            Labeled 8 Bit BGR color image
        """

        status, img = self.get_image()

        if status:
            for o in self.objects:
                cv2.rectangle(
                    img,
                    (o.x, o.y),
                    (o.x + o.width, o.y + o.height),
                    o.color_from_attributes(),
                    2,
                )
        return img

    def get_disparity_image(self):
        """
        Method loading the disparity image

        Returns:
            np.array: disparity image in float

        """

        # quantization
        scale = 1.0 / 16.0
        # load raw image
        if os.path.isfile(self.disp_file_path):
            img = cv2.imread(self.disp_file_path, cv2.IMREAD_UNCHANGED)
        else:
            logging.error("Disparity Image {} not found. Please check image file paths!".format(self.disp_file_path))
            sys.exit(1)

        # do the magic
        img[img == 65535] = 0
        img &= 0x0FFF
        # convert to float
        img = img.astype(np.float32)
        # scale
        np.multiply(img, scale, out=img, casting="unsafe")
        return img

    def visualize_disparity_image(self):
        """
        Method loading the disparity image for visualization

        Returns:
            Disparity image only (!!!) for visualization. Note that the
            true disparity values are modified!!!
        """
        # get disparity image
        img = self.get_disparity_image()
        # rescale to full dynamic
        max = np.max(img)
        min = np.min(img)
        img = img * float((255.0 / float((max - min))))
        img = np.uint8(img)
        # colorize to 3 channels
        img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
        return img

    def map_labels_to_disparity_image(self, calibration: CalibrationData):
        """
        Method loading the labels mapped to the disparity image
        at the correct position

        Args:
            calibration(Calibration: calibration including
            intrinsic, extrinsic, ... calibration matrices
        Returns:
        Label list ([x1, y1, w1, h1], [x2, y2, w2, h2], ...)

        """

        pts_undistorted = []

        for labels in self.objects:
            # not rectified coordinates
            pt_distorted = np.array(
                [
                    [float(labels.x),
                     float(labels.y)],
                    [float(labels.x + labels.width),
                     float(labels.y + labels.height)],
                ]
            )
            pt_distorted = pt_distorted[:, np.newaxis, :]

            # rectify points
            pt_undistorted = cv2.undistortPoints(
                pt_distorted,
                calibration.intrinsic_calibration.intrinsic_matrix,
                calibration.distortion_calibration.distortion_matrix,
                R=calibration.rectification_matrix.rectification_matrix,
                P=calibration.projection_matrix.projection_matrix,
            )

            # get new coords
            x = pt_undistorted[0][0][0]
            y = pt_undistorted[0][0][1]
            w = pt_undistorted[1][0][0] - pt_undistorted[0][0][0]
            h = pt_undistorted[1][0][1] - pt_undistorted[0][0][1]

            # binning in x and y (camera images were binned before
            # disparity calculation)
            rect = [x / 2.0, y / 2.0, w / 2.0, h / 2.0]
            pts_undistorted.append(copy.copy(rect))

        return pts_undistorted


class DriveuDatabase:
    """
    Class describing the DriveU Dataset containing a list of images

    Attributes:
        images (List of DriveuImage)  All images of the dataset
        file_path (string):           Path of the dataset (.json)
    """

    def __init__(self, file_path):
        self.images = []
        self.file_path = file_path

    def open(self, data_base_dir: str = ""):
        """
        Method loading the dataset

        Args:
            data_base_dir(str): Base path where images are stored, optional
            if image paths in json are outdated
        """

        if os.path.exists(self.file_path) is not None:
            label_file_extension = os.path.splitext(self.file_path)[1]
            if label_file_extension == ".json":
                logging.info("Opening DriveuDatabase from file: {}"
                            .format(self.file_path))
                with open(self.file_path, "r") as fp:
                    images = json.load(fp)
            elif label_file_extension == ".yml":
                logging.exception("Yaml support is deprecated. Either use the new .json label files (from download URL received after registration) or checkout <git checkout v1.0> to parse yaml")
                sys.exit(1)
                return False
            else:
                logging.exception("Label file with extension {} not supported. Please use json!".format(label_file_extension))
                sys.exit(1)
                return False
        else:
            logging.exception(
                "Opening DriveuDatabase from File: {} "
                "failed. File or Path incorrect.".format(self.file_path)
            )
            sys.exit(1)
            return False

        for image_dict in images["images"]:
            # parse and store image
            image = DriveuImage()
            image.parse_image_dict(image_dict, data_base_dir)
            self.images.append(image)

        return True
