from __future__ import print_function

__author__ = "Andreas Fregin, Julian Mueller and Klaus Dietmayer"
__maintainer__ = "Julian Mueller"
__email__ = "julian.mu.mueller@daimler.com"

import yaml
import numpy as np

class IntrinsicCalibration():
    """
    Intrinsic calibration
        fx(float):  Focal length in x
        fy(float):  Focal length y
        cx(float):  principal point x
        cy(float):  principal point y
        intrinsic_matrix: Intrinsic camera matrix (opencv format)
    """
    def __init__(self):
        self.fx = 0.
        self.fy = 0.
        self.cx = 0.
        self.cy = 0.
        self.intrinsic_matrix = []

    def load_matrix(self, matrix):
        """
        Method loading intrinsic camera calibration params from
        matrix

        Args:
            matrix: Intrinsic camera matrix
        """
        self.fx = matrix[0][0]
        self.fy = matrix[1][1]
        self.cx = matrix[0][2]
        self.cy = matrix[1][2]
        self.intrinsic_matrix = matrix


class ExtrinsicCalibration():
    """
    Extrinsic Calibration from rear axis to left camera

    Attributes:
        r_i Rotation parts
        ti  Translation parts
        extrinsic_matrix: Extrinsic camera matrix
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

    def load_matrix(self, matrix):
        """
        Method loading extrinsic camera calibration params from
        matrix

        Args:
            matrix: Extrinsic camera matrix
        """
        self.r_11 = matrix[0][0]
        self.r_12 = matrix[0][1]
        self.r_13 = matrix[0][2]
        self.r_21 = matrix[1][0]
        self.r_22 = matrix[1][1]
        self.r_23 = matrix[1][2]
        self.r_31 = matrix[2][0]
        self.r_32 = matrix[2][1]
        self.r_33 = matrix[2][2]
        self.tx = matrix[0][3]
        self.ty = matrix[1][3]
        self.tz = matrix[2][3]

        self.extrinsic_matrix = matrix


class RectificationMatrix():
    """
    Rectification matrix

    Attributes:
        r_i Rectification params
        rectification_matrix: Rectification matrix
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
        self.rectification_matrix = []

    def load_matrix(self, matrix):
        """
        Method loading rectification params from
        matrix

        Args:
            matrix: Extrinsic camera matrix
        """
        self.r_11 = matrix[0][0]
        self.r_12 = matrix[0][1]
        self.r_13 = matrix[0][2]
        self.r_21 = matrix[1][0]
        self.r_22 = matrix[1][1]
        self.r_23 = matrix[1][2]
        self.r_31 = matrix[2][0]
        self.r_32 = matrix[2][1]
        self.r_33 = matrix[2][2]

        self.rectification_matrix = matrix


class ProjectionMatrix():
    """
    Projection matrix

    Attributes:
        fx(float):  Focal length in x
        fy(float):  Focal length y
        cx(float):  principal point x
        cy(float):  principal point y
        tx(float):  Translation x
        ty(float):  Translation y
        baseline(float): Baseline(stereo cam)
        projection_matrix: Projection matrix
    """
    def __init__(self):
        self.fx = 0.
        self.fy = 0.
        self.cx = 0.
        self.cy = 0.
        self.tx = 0.
        self.ty = 0.
        self.baseline = 0.
        self.projection_matrix = []

    def load_matrix(self, matrix):
        """
        Method loading projection params from
        matrix

        Args:
            matrix: Projection matrix
        """
        self.fx = matrix[0][0]
        self.fy = matrix[1][1]
        self.cx = matrix[0][2]
        self.cy = matrix[1][2]
        self.tx = matrix[0][3]
        self.ty = matrix[1][3]
        self.baseline = matrix[0][3] / (-1. * matrix[0][0])

        self.projection_matrix = matrix


class DistortionCalibration():
    """
    Distortion Calibration

    Attributes:
        k1,k2,k3 (float):  radial distortion
        p1, p2    tangential distortion
    """
    def __init__(self):
        self.k1 = 0.
        self.k2 = 0.
        self.k3 = 0.
        self.p1 = 0.
        self.p2 = 0.
        self.distortion_matrix = []

    def load_matrix(self, matrix):
        """
        Method loading distortion params from
        matrix

        Args:
            matrix: Distortion matrix
        """
        self.k1 = matrix[0][0]
        self.k2 = matrix[0][1]
        self.p1 = matrix[0][2]
        self.p2 = matrix[0][3]
        self.k3 = matrix[0][4]

        self.distortion_matrix = matrix


class CalibrationData():
    """
    Calibration class containing intrinsic, extrinsic, distortion,
    projection and rectification calibration

    Attributes:
        intrinsic_calibration:  Intrinsic camera calibration
        extrinsic_calibration:  Extrinsic camera calibration
        distortion_calibration: Distortion calibration
        projection_matrix:      Projection matrix
        rectification_matrix:   Rectification matrix
    """
    def __init__(self):
        self.intrinsic_calibration = IntrinsicCalibration()
        self.extrinsic_calibration = ExtrinsicCalibration()
        self.distortion_calibration = DistortionCalibration()
        self.projection_matrix = ProjectionMatrix()
        self.rectification_matrix = RectificationMatrix()

    def load_calibration_matrix(self, path):
        """
        Loads calibration matrix from .yml file.

        Args:
            path: path to yml-matrix

        Returns:
            matrix

        """
        skip_lines = 2
        with open(path) as infile:
            for i in range(skip_lines):
                _ = infile.readline()
            data = yaml.load(infile)
            matrix = np.reshape(data['data'], (data['rows'], data['cols']))

        return matrix

    def load_intrinsic_matrix(self, path: str):
        """
        This method loads the intrinsic matrix from path
        Args:
            path(str): Path to intrinsic matrix (yml)
        Returns:
            np.array: Intrinsic matrix
        """
        matrix = self.load_calibration_matrix(path)
        self.intrinsic_calibration.load_matrix(matrix)
        return self.intrinsic_calibration.intrinsic_matrix

    def load_extrinsic_matrix(self, path):
        """
        This method loads the extrinsic matrix from path
        Args:
            path(str): Path to extrinsic matrix (yml)
        Returns:
            np.array: Extrinsic matrix
        """
        matrix = self.load_calibration_matrix(path)
        self.extrinsic_calibration.load_matrix(matrix)
        return self.extrinsic_calibration.extrinsic_matrix

    def load_distortion_matrix(self, path):
        """
        This method loads the distortion matrix from path
        Args:
            path(str): Path to distortion matrix (yml)
        Returns:
            np.array: Distortion matrix
        """
        matrix = self.load_calibration_matrix(path)

        self.distortion_calibration.load_matrix(matrix)
        return self.distortion_calibration.distortion_matrix

    def load_projection_matrix(self, path):
        """
        This method loads the projection matrix from path
        Args:
            path(str): Path to projection matrix (yml)
        Returns:
            np.array: Projection matrix
        """
        matrix = self.load_calibration_matrix(path)

        self.projection_matrix.load_matrix(matrix)
        return self.projection_matrix.projection_matrix

    def load_rectification_matrix(self, path):
        """
        This method loads the rectification matrix from path
        Args:
            path(str): Path to rectification matrix (yml)
        Returns:
            np.array: Rectification matrix
        """
        matrix = self.load_calibration_matrix(path)

        self.rectification_matrix.load_matrix(matrix)
        return self.rectification_matrix.rectification_matrix
