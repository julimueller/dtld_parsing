#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include "compand.h"
#include <jsoncpp/json/json.h>
#include "driveu_calibration.h"
#include "driveu_vehicle_data.h"

#ifdef OpenCV_FOUND
#include <opencv2/opencv.hpp>
#endif

struct DriveuAttribute
{

    DriveuAttribute() : m_name_(""), m_class_(""){};
    DriveuAttribute(const std::string &n, const std::string &c) : m_name_(n), m_class_(c){};
    /// Attribute name [e.g. "state"]
    std::string m_name_;
    /// Attribute class [e.g. "red"]
    std::string m_class_;
};

class DriveuObject
{

public:
    /// Bounding box upper left x coordinate
    int m_x_;
    /// Bounding box upper left y coordinate
    int m_y_;
    /// Bounding box width
    int m_width_;
    /// Bounding box height
    int m_height_;
    /// Bounding box unique identity value
    int m_unique_id_;
    /// Bounding box track identity (same real world instances in different images)
    std::string m_track_id_;

    /// Label direction, can be ["front", "right", "left", "back"]
    DriveuAttribute m_direction_;
    /// Label relevance, can be ["relevant", "not_relevant"]
    DriveuAttribute m_relevance_;
    /// Label occlusion, can be ["occluded", "not_occluded"]. Occlusion includes truncation
    DriveuAttribute m_occlusion_;
    /// Label orientation, can be ["vertical", "horizontal"]
    DriveuAttribute m_orientation_;
    /// Label aspects, can be ["one_aspect", "two_aspects", "three_aspects", "four_aspects", "unknown"]
    DriveuAttribute m_aspects_;
    /// Label states, can be ["red", "green", "yellow", "red_yellow", "off", "unknown"]
    DriveuAttribute m_state_;
    /// Label pictograms, can be ["circle", "arrow_straight", "arrow_left", "arrow_right", "pedestrian", "bicycle", "tram", "arrow_straight_left", "unknown"]
    DriveuAttribute m_pictogram_;

    DriveuObject() : m_x_(0), m_y_(0), m_width_(0), m_height_(0), m_unique_id_(0), m_track_id_(""), m_direction_("direction", "unknown"), m_relevance_("relevance", "unknown"), m_occlusion_("occlusion", "unknown"), m_orientation_("orientation", "unknown"), m_aspects_("aspects", "unknown"), m_state_("state", "unknown"), m_pictogram_("pictogram", "unknown"){};

    DriveuObject(const int x, const int y, const int width, const int height, const int uid, const std::string &track_id, const std::string &direction, const std::string &relevance, const std::string &occlusion, const std::string &orientation, const std::string &aspects, const std::string &state, const std::string &pictogram) : m_x_(x), m_y_(y), m_width_(width), m_height_(height), m_unique_id_(uid), m_track_id_(track_id), m_direction_("direction", direction), m_relevance_("relevance", relevance), m_occlusion_("occlusion", occlusion), m_orientation_("orientation", orientation), m_aspects_("aspects", aspects), m_state_("state", state), m_pictogram_("pictogram", pictogram){};

    /**
     * @brief               Parses label dict from json file
     * @param object_dict   Object dictionary read from json file
     * @returns             Success
     * */
    bool parseObjectDict(const Json::Value &object_dict);

#ifdef OpenCV_FOUND
    /**
     * @brief               Returns label bbox as cv::Rect
     * @returns             Label bbox (cv::Rect)
     * */
    cv::Rect getRect() const;
    /**
     * @brief               Returns color for bounding box based on state
     * @returns             Label state as 3 channel color for visualization
     * */
    cv::Scalar colorFromAttribute() const;
#endif
};

class DriveuImage
{

public:
    DriveuImage() : m_file_path_(""), m_disp_file_path_(""), m_timestamp_(0.), m_vehicle_data_(VehicleData()), m_objects_({}){};

    /// File path of left color image
    std::string m_file_path_;
    /// File path of disparity image
    std::string m_disp_file_path_;
    /// Timestamp of frame
    double m_timestamp_;
    /// Additional sensor data at current frame
    VehicleData m_vehicle_data_;
    /// List of labels in frame
    std::vector<DriveuObject> m_objects_;
    /**
     * @brief               Parses image dictionary from json label file
     * @param image_dict    Image dictionary from json file
     * @param data_base_dir Optional base directory of image data (if differing from label file image paths)
     * @returns             Success
     * */
    bool parseImageDict(const Json::Value &image_dict, const std::string &data_base_dir);

     /**
     * @brief               Modifys image base path
     * @param data_base_dir Optional base directory of image data (if differing from label file image paths)
     * @param file_path     Initial absolute image path from label file
     * * @returns           Modified absolute image path
     * */
    std::string modifyImagePath(const std::string &data_base_dir, const std::string &file_path) const;

#ifdef OpenCV_FOUND
    /**
     * @brief       Returning 8 bit left color image (BGR channel order)
     * @returns     cv::Mat
     * */
    cv::Mat getImage() const;
    /**
     * @brief    Returning 16 bit left color image (BGR channel order)
     * @returns  cv::Mat
     * */
    cv::Mat getImage16Bit(Decompand &decompand);
    /**
     * @brief           return disparity image as float, where each pixel contains the disparity in pixels. Please note, that the disparity image has a smaller resolution than the original image and for the lower 144 pixels no disparity values are available
     * @param decompand Decompanding instance needed for converting 12 ->16 bit.
     * @returns         cv::Mat 32FC1
     * */
    cv::Mat getDisparityImage() const;
    /**
     * @brief   returns camera image with visualized bounding boxes colorized by their labeled state
     * @returns cv::Mat in BGR channel order
     * */
    cv::Mat getLabeledImage() const;
    /**
     * @brief   Colorizes a 1 channel disparity image and returns it as a 3 channel image. Please note, that the pixel values cannot be used for 3D reconstruction anymore, use getDisparityImage() for this purpose!
     * @returns cv::Mat in 3 channelBGR channel order
     * */
    void visualizeDisparityImage(cv::Mat &dispMat) const;
    /**
     * @brief       Returns all labels of the acutal image in disparity image           coordinates. Original label coordinates are first rectified and then mapped via binning factors
     * @param calib Calibration of left camera
     * */
    std::vector<cv::Rect> mapLabelsToDisparityImage(CalibrationData &calib) const;
#endif
};

class DriveuDatabase
{

public:
    DriveuDatabase() : m_images_({}){};
    // all images of the dataset
    std::vector<DriveuImage> m_images_;
    /**
     * @brief           Loads database from .json labelfile
     * @param path      Label file path
     * @param base_path Optional: If dataset images directory differs from the label files (e.g. moved to another directory) this can be set here. Please note, that the base path refers to the top directory of the original dataset structure.
     * */
    bool open(const std::string &path, const std::string &base_path);
};
