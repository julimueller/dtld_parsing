from __future__ import print_function

__author__ = "Andreas Fregin, Julian Mueller and Klaus Dietmayer"
__maintainer__ = "Julian Mueller"
__email__ = "julian.mu.mueller@daimler.com"

import cv2
import yaml
import numpy as np
import os.path
import copy
import time
import argparse
import logging

from calibration import (
  IntrinsicCalibration,
  ExtrinsicCalibration,
  RectificationMatrix,
  ProjectionMatrix,
  DistortionCalibration,
  CalibrationData
)


class DriveuObject():
  """ Class describing a label object in the dataset by rectangle

  Attributes:
    x (int):          X coordinate of upper left corner of bouding box label
    y (int):          Y coordinate of upper left corner of bouding box label
    width (int):      Width of bounding box label
    height (int):     Height of bounding box label
    class_id (int):   6 Digit class idenntity of bounding box label (Digit explanation see documentation pdf)
    uniqie_id (int):  Unique ID of the object
    track_id (string) Track ID of the object (representing one real-world TL instance)

  """

  def __init__(self):
    self.x = 0
    self.y = 0
    self.width = 0
    self.height = 0
    self.class_id = 0
    self.unique_id = 0
    self.track_id = 0


  def colorFromClassId(self):
    """ Color for bounding box visualization

    Returns:
      Color-Vector (BGR) for traffic light visualization

    """
     #Second last digit indicates state/color
    if str(self.class_id)[-2] == "1":
      return (0,0,255)
    elif str(self.class_id)[-2] == "2":
      return (0,255,255)
    elif str(self.class_id)[-2] == "3":
      return (0,165,255)
    elif str(self.class_id)[-2] == "4":
      return (0,255,0)
    else:
      return (255,255,255)

class VehicleData():
  """ Class describing the vehicle data corresponding to the timestamp of the color image

  Attributes:
    velocity (float):   Velocity of the vehicle in m/s
    yaw_rate (float):   Yaw Rate of the vehicle in rad/s, counterclock-wise
    longitude (float):  GPS Longitude in Degree
    latitude (float):   GPS latitude in Degree

  """

  def __init__(self):
    self.velocity = 0.
    self.yaw_rate = 0.
    self.longitude = 0.
    self.latitude = 0.


class DriveuImage():
  """ Class describing one image in the DriveU Database

  Attributes:
    file_path (string):         Path of the left camera image
    disp_file_path (string):    Path of the corresponding disparity image
    timestamp (float):          Timestamp of the image
    vehicle_data (VehicleData)  Vehicle Data at that timestamp (or approximately)
    objects (DriveuObject)      Labels in that image
    """
  def __init__(self):
    self.file_path = ''
    self.disp_file_path = ''
    self.timestamp = 0
    self.vehicle_data = VehicleData()
    self.objects = []

  def getImage(self):
    """ Method loading the left unrectified color image in 8 Bit

    Returns:
      8 Bit BGR color image

    """
    if os.path.isfile(self.file_path):
      """Load image from file path, do debayering and shift"""
      img = cv2.imread(self.file_path, cv2.IMREAD_UNCHANGED)
      img = cv2.cvtColor(img, cv2.COLOR_BAYER_GB2BGR)
      # Images are saved in 12 bit raw -> shift 4 bits
      img = np.right_shift(img, 4)
      img = img.astype(np.uint8)

      return True, img

    else:

      logging.exception("Image {} not found".format(self.file_path))

      return False, img

  def getLabeledImage(self):
    """Method loading the left unrectified color image and drawing labels in it

    Returns:
      Labeled 8 Bit BGR color image

    """

    status, img = self.getImage()


    for o in self.objects:
      cv2.rectangle(img, (o.x, o.y), (o.x + o.width, o.y + o.height), o.colorFromClassId(), 2)

    return img

  def getDisparityImage(self):
    """Method loading the disparity image

    Returns:
      Disparity image in float

    """

    scale = 1. / 16.

    img = cv2.imread(self.disp_file_path, cv2.IMREAD_UNCHANGED)

    img [img == 65535] = 0
    img &= 0x0FFF

    img = img.astype(np.float32)

    np.multiply(img, scale, out= img, casting="unsafe")

    return img

  def visualizeDisparityImage(self):
    """Method loading the disparity image for visualization

    Returns:
      Disparity image only (!!!) for visualization

    """
    img = self.getDisparityImage()
    max = np.max(img)
    min = np.min(img)
    img = img * float((255./float((max-min))))
    img = np.uint8(img)

    img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
    return img

  def mapLabelsToDisparityImage(self, calibration):
    """Method loading the labels mapped to the disparity image at the correct position

    Returns:
      Label list ([x1, y1, w1, h1], [x2, y2, w2, h2], ...)

    """

    pts_undistorted = []

    for labels in self.objects:
      pt_distorted = np.array([[float(labels.x), float(labels.y)], [float(labels.x + labels.width), float(labels.y + labels.height)]])
      pt_distorted = pt_distorted[:, np.newaxis, :]

      pt_undistorted = cv2.undistortPoints(pt_distorted,calibration.intrinsic_calibration.intrinsic_matrix, calibration.distortion_calibration.distortion_matrix, R = calibration.rectification_matrix.rectification_matrix, P = calibration.projection_matrix.projection_matrix)
      x = pt_undistorted[0][0][0]
      y = pt_undistorted[0][0][1]
      w = pt_undistorted[1][0][0] - pt_undistorted[0][0][0]
      h = pt_undistorted[1][0][1] - pt_undistorted[0][0][1]

      # binning in x and y
      rect = [x/2., y/2., w/2., h/2.]
      pts_undistorted.append(copy.copy(rect))

    return pts_undistorted


class DriveuDatabase():
  """ Class describing the DriveU Dataset

  Attributes:
    images (List of DriveuImage)  All images of the dataset
    file_path (string):           Path of the dataset (.yml)
    """
  def __init__(self, file_path):
    self.images = []
    self.file_path = file_path

  def open(self, data_base_dir):
    """Method loading the dataset

    """

    if os.path.exists(self.file_path) is not None:
      logging.info('Opening DriveuDatabase from file: {}'.format(self.file_path))
      images = yaml.load(open(self.file_path, 'rb').read())

      for i, image_dict in enumerate(images):

        image = DriveuImage()
        if data_base_dir != '':
            inds = [i for i, c in enumerate(image_dict['path']) if c == '/']
            image.file_path = data_base_dir + '/' + image_dict['path'][inds[-4]:]
            inds = [i for i, c in enumerate(image_dict['disp_path']) if c == '/']
            image.disp_file_path = data_base_dir + '/' + image_dict['disp_path'][inds[-4]:]
        else:
            image.file_path = image_dict['path']
            image.disp_file_path = image_dict['disp_path']
        image.timestamp = image_dict['time_stamp']
        image.vehicle_data.velocity = image_dict['velocity']
        image.vehicle_data.yaw_rate = image_dict['yaw_rate']
        image.vehicle_data.longitude = image_dict['longitude']
        image.vehicle_data.latitude = image_dict['latitude']

        for o in image_dict['objects']:

          object = DriveuObject()

          object.x = o['x']
          object.y = o['y']
          object.width = o['width']
          object.height = o['height']
          object.class_id = o['class_id']
          object.unique_id = o['unique_id']
          object.track_id = o['track_id']

          cpy = copy.copy(object)

          image.objects.append(cpy)

        copy_image = copy.copy(image)
        self.images.append(copy_image)


    else:
      logging.exception('Opening DriveuDatabase from File: {} '
                        'failed. File or Path incorrect.'.format(self.file_path))

def parse_args():

  parser = argparse.ArgumentParser()
  parser.add_argument('--label_file', default='')
  parser.add_argument('--calib_dir', default='')
  parser.add_argument('--data_base_dir', default='')
  return parser.parse_args()


def main(args):

  database = DriveuDatabase(args.label_file)

  calibration = CalibrationData()
  intrinsic_left = calibration.load_intrinsic_matrix(args.calib_dir + '/intrinsic_left.yml')
  rectification_left = calibration.load_rectification_matrix(args.calib_dir + '/rectification_left.yml')
  projection_left = calibration.load_projection_matrix(args.calib_dir + '/projection_left.yml')
  extrinsic = calibration.load_extrinsic_matrix(args.calib_dir + '/extrinsic.yml')
  distortion_left = calibration.load_distortion_matrix(args.calib_dir + '/distortion_left.yml')

  database.open(args.data_base_dir)

  logging.info("Intrinsic Matrix: {}".format(intrinsic_left))
  logging.info("Extrinsic Matrix:".format(extrinsic))
  logging.info("Projection Matrix:".format(projection_left))
  logging.info("Rectification Matrix:".format(rectification_left))
  logging.info("Distortion Matrix:".format(distortion_left))

  for idx_d, img in enumerate(database.images):

    img_disp = img.visualizeDisparityImage()
    rects = img.mapLabelsToDisparityImage(calibration)

    for rect in rects:
      cv2.rectangle(img_disp, (int(rect[0]), int(rect[1])), (int(rect[0]) + int(rect[2]), int(rect[1]) + int(rect[3])), (255,255,255), 2)

    img_color = img.getLabeledImage()
    img_color = cv2.resize(img_color, (1024, 440))

    img_concat = np.concatenate((img_color, img_disp), axis=1)
    cv2.imshow("DTLD_visualized", img_concat)
    cv2.waitKey(1)

if __name__ == '__main__':
  main(parse_args())


