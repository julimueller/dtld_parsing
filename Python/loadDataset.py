import cv2
import yaml 
import numpy as np
import os.path
import copy
import time

__author__ = "Andreas Fregin, Julian Mueller and Klaus Dietmayer"
__maintainer__ = "Julian Mueller"
__email__ = "julian-2.mueller@uni-ulm.de"

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

      print "Image " + str(self.file_path) + "not found"

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

      pt_undistorted = cv2.undistortPoints(pt_distorted,calibration.intrinsic_matrix.intrinsic_matrix, calibration.distortion_matrix.distortion_matrix, R = calibration.rectification_matrix.rectification_matrix, P = calibration.projection_matrix.projection_matrix)
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

  def open(self):
    """Method loading the dataset

    """

    if os.path.exists(self.file_path) is not None:
      print 'Opening DriveuDatabase from File: ' + str(self.file_path)
      images = yaml.load(open(self.file_path, 'rb').read())

      for i, image_dict in enumerate(images):

        image = DriveuImage()
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
      print 'Opening DriveuDatabase from File: ' + str(self.file_path) + 'failed. File or Path incorrect.'

class IntrinsicCalibration():
  """ Class describing the Intrinsic Calibration

  Attributes:
    fx  Focal length x
    fy  Focal length y
    cx  principal point x
    cy  principal point y
    intrinsic_matrix Intrinsic camera matrix (opencv format)

    """
  def __init__(self):
    self.fx = 0.
    self.fy = 0.
    self.cx = 0.
    self.cy = 0.
    self.intrinsic_matrix = []

class ExtrinsicCalibration():
  """ Class describing the Extrinsic Calibration from rear axis to left camera

  Attributes:
    r_i Rotation parts
    ti  Translation parts
  """
  def __init__(self):
    self.r_11 = 0.
    self.r_12 = 0.
    self.r_13 = 0.
    self.r_21 = 0.
    self.r_22 = 0.
    self.r_23 = 0.
    self.r_31 = 0.
    self.r_32 = 0.
    self.r_33 = 0.
    self.tx = 0.
    self.ty = 0.
    self.tz = 0.
    self.extrinsic_matrix = []

class DistortionCalibration():
  """ Class describing the Distortion

  Attributes:
    k1,k2,k3  radial distortion
    p1, p2    tangential distortion
"""
  def __init__(self):
    self.k1 = 0.
    self.k2 = 0.
    self.k3 = 0.
    self.p1 = 0.
    self.p2 = 0.
    self.distortion_matrix = []

class ProjectionMatrix():
  def __init__(self):
    self.fx = 0.
    self.fy = 0.
    self.cx = 0.
    self.cy = 0.
    self.tx = 0.
    self.ty = 0.
    self.baseline = 0.
    self.projection_matrix = []


class RectificationMatrix():
  def __init__(self):
    self.r_11 = 0.
    self.r_12 = 0.
    self.r_13 = 0.
    self.r_21 = 0.
    self.r_22 = 0.
    self.r_23 = 0.
    self.r_31 = 0.
    self.r_32 = 0.
    self.r_33 = 0.
    self.rectification_matrix = []

class CalibrationData():

  def __init__(self):
    self.intrinsic_matrix = IntrinsicCalibration()
    self.extrinsic_matrix = ExtrinsicCalibration()
    self.distortion_matrix = DistortionCalibration()
    self.projection_matrix = ProjectionMatrix()
    self.rectification_matrix = RectificationMatrix()


  def loadYmlMatrix(self, path):
    """Returns calibration matrix loaded from .yml file."""

    skip_lines = 2
    with open(path) as infile:
        for i in range(skip_lines):
            _ = infile.readline()
        data = yaml.load(infile)
        matrix = np.reshape(data['data'],(data['rows'],data['cols']))

    return matrix, data['rows'], data['cols']

  def loadIntrinsicMatrix(self, path):
    matrix, rows, cols = self.loadYmlMatrix(path)

    if (rows != 3 or cols != 3):
      print "Intrinsic camera matrix has not shape 3x3, please check input file!"
      return

    self.intrinsic_matrix.fx = matrix[0][0]
    self.intrinsic_matrix.fy = matrix[1][1]
    self.intrinsic_matrix.cx = matrix[0][2]
    self.intrinsic_matrix.cy = matrix[1][2]

    self.intrinsic_matrix.intrinsic_matrix = matrix

    return matrix

  def loadExtrinsicMatrix(self, path):
    matrix, rows, cols = self.loadYmlMatrix(path)

    if (rows != 3 or cols != 4):
      print "Extrinsic camera matrix has not shape 3x4, please check input file!"
      return

    self.extrinsic_matrix.r_11 = matrix[0][0]
    self.extrinsic_matrix.r_12 = matrix[0][1]
    self.extrinsic_matrix.r_13 = matrix[0][2]
    self.extrinsic_matrix.r_21 = matrix[1][0]
    self.extrinsic_matrix.r_22 = matrix[1][1]
    self.extrinsic_matrix.r_23 = matrix[1][2]
    self.extrinsic_matrix.r_31 = matrix[2][0]
    self.extrinsic_matrix.r_32 = matrix[2][1]
    self.extrinsic_matrix.r_33 = matrix[2][2]
    self.extrinsic_matrix.tx = matrix[0][3]
    self.extrinsic_matrix.ty = matrix[1][3]
    self.extrinsic_matrix.tz = matrix[2][3]

    self.extrinsic_matrix.extrinsic_matrix = matrix

    return matrix

  def loadDistortionMatrix(self, path):
    matrix, rows, cols = self.loadYmlMatrix(path)

    if (rows != 1 or cols != 5):
      print "Distortion matrix has not shape 1x5, please check input file!"
      return

    self.distortion_matrix.k1 = matrix[0][0]
    self.distortion_matrix.k2 = matrix[0][1]
    self.distortion_matrix.p1 = matrix[0][2]
    self.distortion_matrix.p2 = matrix[0][3]
    self.distortion_matrix.k3 = matrix[0][4]

    self.distortion_matrix.distortion_matrix = matrix

    return matrix

  def loadProjectionMatrix(self, path):
    matrix, rows, cols = self.loadYmlMatrix(path)

    if (rows != 3 or cols != 4):
      print "Projection matrix has not shape 3x4, please check input file!"
      return

    self.projection_matrix.fx = matrix[0][0]
    self.projection_matrix.fy = matrix[1][1]
    self.projection_matrix.cx = matrix[0][2]
    self.projection_matrix.cy = matrix[1][2]
    self.projection_matrix.tx = matrix[0][3]
    self.projection_matrix.ty = matrix[1][3]
    self.projection_matrix.baseline = matrix[0][3] / (-1. * matrix[0][0])

    self.projection_matrix.projection_matrix = matrix

    return matrix

  def loadRectificationMatrix(self, path):
    matrix, rows, cols = self.loadYmlMatrix(path)

    if (rows != 3 or cols != 3):
      print "Rectification matrix has not shape 3x3, please check input file!"
      return

    self.rectification_matrix.r_11 = matrix[0][0]
    self.rectification_matrix.r_12 = matrix[0][1]
    self.rectification_matrix.r_13 = matrix[0][2]
    self.rectification_matrix.r_21 = matrix[1][0]
    self.rectification_matrix.r_22 = matrix[1][1]
    self.rectification_matrix.r_23 = matrix[1][2]
    self.rectification_matrix.r_31 = matrix[2][0]
    self.rectification_matrix.r_32 = matrix[2][1]
    self.rectification_matrix.r_33 = matrix[2][2]

    self.rectification_matrix.rectification_matrix = matrix

    return matrix


if __name__ == '__main__':

  database = DriveuDatabase('/scratch/fs2/DriveUDataset/Essen_all.yml')

  calibration = CalibrationData()
  intrinsic_left = calibration.loadIntrinsicMatrix('/home/muejul3/git_repos/dtld_parsing/Calibration/intrinsic_left.yml')
  rectification_left = calibration.loadRectificationMatrix('/home/muejul3/git_repos/dtld_parsing/Calibration/rectification_left.yml')
  projection_left = calibration.loadProjectionMatrix('/home/muejul3/git_repos/dtld_parsing/Calibration/projection_left.yml')
  extrinsic = calibration.loadExtrinsicMatrix('/home/muejul3/git_repos/dtld_parsing/Calibration/extrinsic.yml')
  distortion_left = calibration.loadDistortionMatrix('/home/muejul3/git_repos/dtld_parsing/Calibration/distortion_left.yml')

  database.open()

  print calibration.intrinsic_matrix.fx
  print calibration.intrinsic_matrix.intrinsic_matrix

  print "Intrinsic Matrix:\n\n" + str(intrinsic_left) + "\n"
  print "Extrinsic Matrix:\n\n" + str(extrinsic) + "\n"
  print "Projection Matrix:\n\n"  + str(projection_left) + "\n"
  print "Rectification Matrix:\n\n" + str(rectification_left) + "\n"
  print "Distortion Matrix:\n\n" + str(distortion_left) + "\n"

  for idx_d, img in enumerate(database.images):

    img_disp_orig = img.getDisparityImage()
    img_disp = img.visualizeDisparityImage()

    rects = img.mapLabelsToDisparityImage(calibration)

    for rect in rects:

      cv2.rectangle(img_disp, (int(rect[0]), int(rect[1])), (int(rect[0]) + int(rect[2]), int(rect[1]) + int(rect[3])), (255,255,255), 2)
      img_color = img.getLabeledImage()

    img_color = cv2.resize(img_color, (1024, 440))

    img_concat = np.concatenate((img_color, img_disp), axis=1)
    cv2.imshow("DTLD_visualized", img_concat)
    cv2.waitKey(1)

    


