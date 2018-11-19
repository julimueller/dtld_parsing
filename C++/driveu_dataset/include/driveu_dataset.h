#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include "compand.h"


#ifdef OpenCV_FOUND
#include <opencv2/opencv.hpp>
#endif



class DriveuObject {

public:

    DriveuObject();
    ~DriveuObject();

    int x_;
    int y_;
    int width_;
    int height_;
    int class_id_;
    int unique_id_;
    std::string track_id_;

    #ifdef OpenCV_FOUND
      cv::Rect getRect();
      cv::Scalar colorFromClassId();
    #endif

};

class VehicleData {

public:

    VehicleData();
    ~VehicleData();

    double velocity_;
    double yaw_rate_;
    double longitude_;
    double latitude_;
};


class IntrinsicCalibration {

public:

    float fx_;
    float fy_;
    float cx_;
    float cy_;

    #ifdef OpenCV_FOUND
        cv::Mat cv_intrinsic_matrix_;
    #endif

    std::vector<std::vector<float> > intrinsic_matrix_;

};

class DistortionCalibration {

public:

    float k1_;
    float k2_;
    float k3_;
    float p1_;
    float p2_;

    #ifdef OpenCV_FOUND
        cv::Mat cv_distortion_matrix_;
    #endif

    std::vector<std::vector<float> > distortion_matrix_;


};

class ProjectionMatrix {

public:

    float fx_;
    float fy_;
    float cx_;
    float cy_;
    float tx_;
    float ty_;
    float baseline_;

    #ifdef OpenCV_FOUND
        cv::Mat cv_projection_matrix_;
    #endif

    std::vector<std::vector<float> > projection_matrix_;

};

class RectificationMatrix {

public:

    float r_11_;
    float r_12_;
    float r_13_;
    float r_21_;
    float r_22_;
    float r_23_;
    float r_31_;
    float r_32_;
    float r_33_;

    #ifdef OpenCV_FOUND
        cv::Mat cv_rectification_matrix_;
    #endif

    std::vector<std::vector<float> > rectification_matrix_;


};

class ExtrinsicMatrix {

public:

    float r_11_;
    float r_12_;
    float r_13_;
    float r_21_;
    float r_22_;
    float r_23_;
    float r_31_;
    float r_32_;
    float r_33_;

    float tx_;
    float ty_;
    float tz_;

    #ifdef OpenCV_FOUND
        cv::Mat cv_extrinsic_matrix_;
    #endif

    std::vector<std::vector<float> > extrinsic_matrix_;

};

class CalibrationData {

public:

    CalibrationData();
    ~CalibrationData();

    IntrinsicCalibration intrinsic_matrix;
    DistortionCalibration distortion_matrix;
    ProjectionMatrix projection_matrix;
    RectificationMatrix rectification_matrix;
    ExtrinsicMatrix extrinsic_matrix;


    bool loadIntrinsicMatrix(const std::string &path);
    bool loadProjectionMatrix(const std::string &path);
    bool loadDistortionMatrix(const std::string &path);
    bool loadRectificationMatrix(const std::string &path);
    bool loadExtrinsicMatrix(const std::string &path);

    #ifdef OpenCV_FOUND
        cv::Mat getCvIntrinsicMatrix();
        cv::Mat getCvExtrinsicMatrix();
        cv::Mat getCvProjectionMatrix();
        cv::Mat getCvDistortionMatrix();
        cv::Mat getCvRectificationMatrix();
    #endif

    std::vector<std::vector<float> > getIntrinsicMatrix();
    std::vector<std::vector<float> > getExtrinsicMatrix();
    std::vector<std::vector<float> > getProjectionMatrix();
    std::vector<std::vector<float> > getDistortionMatrix();
    std::vector<std::vector<float> > getRectificationMatrix();

private:

    bool loadYmlMatrix(const std::string &path, int &rows, int &cols, std::vector<float> &data_vec);
    #ifdef OpenCV_FOUND
        bool fillCvMat(cv::Mat &mat, std::vector<float> &vec, const int &rows, const int &cols);
    #endif
    bool fillMat(std::vector<std::vector<float> > &mat, std::vector<float> &vec, const int &rows, const int &cols);

};

class DriveuImage {

public:

    DriveuImage();
    ~DriveuImage();

    std::string file_path_;
    std::string disp_file_path_;

    double timestamp_;
    VehicleData vehicle_data;

    std::vector<DriveuObject> objects;

    Decompand* decomp_;


    #ifdef OpenCV_FOUND
        bool getImage(cv::Mat &imageMat);
        cv::Mat getImage16Bit();
        bool getDisparityImage(cv::Mat &dispMat);
        bool getLabeledImage(cv::Mat &imageMat);
        void visualizeDisparityImage(cv::Mat &dispMat);
        std::vector<cv::Rect> mapLabelsToDisparityImage(CalibrationData &calib);
    #endif
};

class DriveuDatabase {

public:

    DriveuDatabase();
    ~DriveuDatabase();

    std::vector<DriveuImage> images;
    bool open(const std::string &path);

};




