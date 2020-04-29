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

    std::string m_name_;
    std::string m_class_;
};

class DriveuObject
{

public:
    int m_x_;
    int m_y_;
    int m_width_;
    int m_height_;
    int m_unique_id_;
    std::string m_track_id_;

    DriveuAttribute m_direction_;
    DriveuAttribute m_relevance_;
    DriveuAttribute m_occlusion_;
    DriveuAttribute m_orientation_;
    DriveuAttribute m_aspects_;
    DriveuAttribute m_state_;
    DriveuAttribute m_pictogram_;



    DriveuObject() : m_x_(0), m_y_(0), m_width_(0), m_height_(0), m_unique_id_(0), m_track_id_(""), m_direction_("direction", "unknown"), m_relevance_("relevance", "unknown"), m_occlusion_("occlusion", "unknown"), m_orientation_("orientation", "unknown"), m_aspects_("aspects", "unknown"), m_state_("state", "unknown"), m_pictogram_("pictogram", "unknown"){};

    DriveuObject(const int x, const int y, const int width, const int height, const int uid, const std::string &track_id, const std::string &direction, const std::string &relevance, const std::string &occlusion, const std::string &orientation, const std::string &aspects, const std::string &state, const std::string &pictogram) : m_x_(x), m_y_(y), m_width_(width), m_height_(height), m_unique_id_(uid), m_track_id_(track_id), m_direction_("direction", direction), m_relevance_("relevance", relevance), m_occlusion_("occlusion", occlusion), m_orientation_("orientation", orientation), m_aspects_("aspects", aspects), m_state_("state", state), m_pictogram_("pictogram", pictogram){};

    bool parseObjectDict(const Json::Value &object_dict);

#ifdef OpenCV_FOUND
    cv::Rect getRect();
    cv::Scalar colorFromAttribute();
#endif
};

class DriveuImage
{

public:
    DriveuImage();

    std::string m_file_path_;
    std::string m_disp_file_path_;

    double m_timestamp_;
    VehicleData m_vehicle_data_;

    std::vector<DriveuObject> m_objects_;

    Decompand *m_decomp_;

    bool parseImageDict(const Json::Value &image_dict);

#ifdef OpenCV_FOUND
    bool getImage(cv::Mat &imageMat);
    cv::Mat getImage16Bit();
    bool getDisparityImage(cv::Mat &dispMat);
    bool getLabeledImage(cv::Mat &imageMat);
    void visualizeDisparityImage(cv::Mat &dispMat);
    std::vector<cv::Rect> mapLabelsToDisparityImage(CalibrationData &calib);
#endif
};

class DriveuDatabase
{

public:
    DriveuDatabase();

    std::vector<DriveuImage> m_images_;
    bool open(const std::string &path, const std::string &base_path);
};
