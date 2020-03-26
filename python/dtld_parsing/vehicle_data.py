from __future__ import print_function

__author__ = "Andreas Fregin, Julian Mueller and Klaus Dietmayer"
__maintainer__ = "Julian Mueller"
__email__ = "julian.mu.mueller@daimler.com"


class VehicleData():
    """
    Class describing the vehicle data corresponding to the timestamp
    of the color image

    Attributes:
        velocity (float):   Velocity of the vehicle in m/s
        yaw_rate (float):   Yaw Rate of the vehicle in rad/s,
        counterclock-wise
        longitude (float):  GPS Longitude in Degree
        latitude (float):   GPS latitude in Degree

    """
    def __init__(self):
        self.velocity = 0.
        self.yaw_rate = 0.
        self.longitude = 0.
        self.latitude = 0.

    def parse_vehicle_data_dict(self, image_dict: dict):
        """
        Method loading vehicle data from yaml file dict

        Args:
            vehicle_data_dict(dict): vehicle dict read from yml
        """
        self.velocity = image_dict['velocity']
        self.yaw_rate = image_dict['yaw_rate']
        self.longitude = image_dict['longitude']
        self.latitude = image_dict['latitude']
