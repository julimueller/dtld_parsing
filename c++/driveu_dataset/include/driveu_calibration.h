#ifndef DRIVEU_DATASET_DRIVEU_CALIBRATION_H
#define DRIVEU_DATASET_DRIVEU_CALIBRATION_H

#include <string>
#include <vector>
#include <fstream>

#ifdef OpenCV_FOUND
#include <opencv2/opencv.hpp>
#endif

class CameraMatrix
{
public:
    CameraMatrix(const std::string &file_path) : m_camera_matrix_file_path_(file_path){};

protected:
#ifdef OpenCV_FOUND
    /// intrinsic camera matrix
    cv::Mat m_cv_matrix_;
#endif
    const std::string m_camera_matrix_file_path_;
    std::vector<std::vector<float>> m_matrix_;

    /**
     * @brief   Function loads a calibration matrix from .yml file format
     * @param path      Path of YAML file containing matrix
     * @param rows      Returning number of rows of matrix
     * @param cols      Returning number of cols of matrix
     * @param data_vec  Returning matrix as vector of float
     * @return          Success of matrix loading
     */
    bool loadYmlMatrix(const std::string &path, int &rows, int &cols, std::vector<float> &data_vec);

    /**
     * @brief       Fills vector of vector float from vector of float
     * @param mat   Returns vector of vector (opencv independent format)
     * @param vec   Data Vector to be transformed
     * @param rows  Number of rows of matrix
     * @param cols  Number of cols of matrix
     */
    bool fillMat(std::vector<std::vector<float>> &mat, const std::vector<float> &vec, const int rows, const int cols);

#ifdef OpenCV_FOUND
    bool fillCvMat(cv::Mat &mat, const std::vector<float> &vec, const int rows, const int cols);
#endif
};

class IntrinsicMatrix : public CameraMatrix
{

private:
    /// focal length in x-direction
    float m_fx_;
    /// focal length in y-direction
    float m_fy_;
    /// principal point in x-direction
    float m_cx_;
    /// principal point in y-direction
    float m_cy_;

public:
    IntrinsicMatrix(const std::string &file_path) : CameraMatrix(file_path), m_fx_(0.), m_fy_(0.), m_cx_(0.), m_cy_(0.){};

#ifdef OpenCV_FOUND
    /// intrinsic camera matrix
    cv::Mat loadCvMatrix();
#endif
    std::vector<std::vector<float>> loadMatrix();
};

class DistortionMatrix : public CameraMatrix
{
private:
    /// distortion parameter k1
    float m_k1_;
    /// distortion parameter k2
    float m_k2_;
    /// distortion parameter k3
    float m_k3_;
    /// distortion parameter p1
    float m_p1_;
    /// distortion parameter p2
    float m_p2_;

public:
    DistortionMatrix(const std::string &file_path) : CameraMatrix(file_path), m_k1_(0.), m_k2_(0.), m_k3_(0.), m_p1_(0.), m_p2_(0.){};

#ifdef OpenCV_FOUND
    /// distortion matrix
    cv::Mat loadCvMatrix();
#endif
    std::vector<std::vector<float>> loadMatrix();
};

class ProjectionMatrix : public CameraMatrix
{

private:
    /// focal length in x-direction (left cam)
    float m_fx_;
    /// focal length in y-direction (left cam)
    float m_fy_;
    /// principal point in x-direction (left cam)
    float m_cx_;
    /// principal point in y-direction (left cam)
    float m_cy_;
    /// parameters related to the optical center of right cam, see
    /// http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
    /// for more details
    float m_tx_;
    float m_ty_;
    /// stereo camera baseline
    float m_baseline_;

public:
    ProjectionMatrix(const std::string &file_path) : CameraMatrix(file_path), m_fx_(0.), m_fy_(0.), m_cx_(0.), m_cy_(0.), m_tx_(0.), m_ty_(0.), m_baseline_(0.){};

#ifdef OpenCV_FOUND
    /// distortion matrix
    cv::Mat loadCvMatrix();
#endif
    std::vector<std::vector<float>> loadMatrix();
};

class RectificationMatrix : public CameraMatrix
{

private:
    /// rectification matrix aligning the camera coordinates systems to that epipolar lines in both stereo images are parallel
    float m_r_11_;
    float m_r_12_;
    float m_r_13_;
    float m_r_21_;
    float m_r_22_;
    float m_r_23_;
    float m_r_31_;
    float m_r_32_;
    float m_r_33_;

public:
    RectificationMatrix(const std::string &file_path) : CameraMatrix(file_path), m_r_11_(0.), m_r_12_(0.), m_r_13_(0.), m_r_21_(0.), m_r_22_(0.), m_r_23_(0.), m_r_31_(0.), m_r_32_(0.), m_r_33_(0.){};

#ifdef OpenCV_FOUND
    /// distortion matrix
    cv::Mat loadCvMatrix();
#endif
    std::vector<std::vector<float>> loadMatrix();
};

class ExtrinsicMatrix : public CameraMatrix
{

private:
    /// extrinsic camera matrix rotation part
    float m_r_11_;
    float m_r_12_;
    float m_r_13_;
    float m_r_21_;
    float m_r_22_;
    float m_r_23_;
    float m_r_31_;
    float m_r_32_;
    float m_r_33_;
    /// extrinsic camera matrix translation part
    float m_tx_;
    float m_ty_;
    float m_tz_;

public:
    ExtrinsicMatrix(const std::string &file_path) : CameraMatrix(file_path), m_r_11_(0.), m_r_12_(0.), m_r_13_(0.), m_r_21_(0.), m_r_22_(0.), m_r_23_(0.), m_r_31_(0.), m_r_32_(0.), m_r_33_(0.), m_tx_(0.), m_ty_(0.), m_tz_(0.){};

#ifdef OpenCV_FOUND
    /// distortion matrix
    cv::Mat loadCvMatrix();
#endif
    std::vector<std::vector<float>> loadMatrix();
};

#endif //DRIVEU_DATASET_DRIVEU_CALIBRATION_H