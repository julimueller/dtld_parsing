#include "driveu_calibration.h"

bool CameraMatrix::loadYmlMatrix(const std::string &path, int &rows, int &cols, std::vector<float> &data_vec)
{

    std::ifstream inFile;

    std::string line = "initial";

    inFile.open(path.c_str());
    if (!inFile)
    {
        std::cout << "driveuDatabase:\tERROR: Intrinsic matrix file " << path << " not found!" << std::endl
                  << std::endl;
        return false;
    }

    std::string rows_str = "rows: ";
    std::string cols_str = "cols: ";
    std::string data_str = "data: ";
    while (std::getline(inFile, line))
    {

        if (line.find(rows_str) != std::string::npos)
        {
            size_t pos = line.find(rows_str);
            rows = std::stoi(line.substr(pos + rows_str.size(), line.size() - 1));
        }

        else if (line.find(cols_str) != std::string::npos)
        {
            size_t pos = line.find(cols_str);
            cols = std::stoi(line.substr(pos + cols_str.size(), line.size() - 1));
        }

        else if (line.find(data_str) != std::string::npos)
        {
            size_t pos = line.find(data_str);
            std::string data_all = line.substr(pos + data_str.size() + 1, line.size() - (pos + data_str.size() + 1) - 1);

            data_all.erase(std::remove(data_all.begin(), data_all.end(), ' '), data_all.end());
            std::stringstream ss(data_all);

            float i;

            while (ss >> i)
            {
                data_vec.push_back(i);

                if (ss.peek() == ',')
                {
                    ss.ignore();
                }
            }
        }
    }

    return true;
}

#ifdef OpenCV_FOUND
/**
 * @brief CalibrationData::fillCvMat()      Fills cv Mat from vector of float
 * @param mat                               Returns cv::Mat (float)
 * @param vec                               Data Vector to be transformed
 * @param rows                              Number of rows of matrix
 * @param cols                              Number of cols of matrix
 */
bool CameraMatrix::fillCvMat(cv::Mat &mat, const std::vector<float> &vec, const int rows, const int cols)
{

    mat = cv::Mat(rows, cols, CV_32FC1);
    int idx = 0;

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            mat.at<float>(i, j) = vec.at(idx);
            ++idx;
        }
    }

    return true;
}
#endif

/**
 * @brief CalibrationData::fillCvMat()      Fills vector of vector float from vector of float
 * @param mat                               Returns vector of vector (opencv independent format)
 * @param vec                               Data Vector to be transformed
 * @param rows                              Number of rows of matrix
 * @param cols                              Number of cols of matrix
 */
bool CameraMatrix::fillMat(std::vector<std::vector<float>> &mat, const std::vector<float> &vec, const int rows, const int cols)
{

    int idx = 0;

    for (int i = 0; i < rows; ++i)
    {

        std::vector<float> temp;

        for (int j = 0; j < cols; ++j)
        {
            temp.push_back(float(vec.at(idx)));
            ++idx;
        }
        mat.push_back(temp);
    }

    return true;
}
#ifdef OpenCV_FOUND
cv::Mat IntrinsicMatrix::loadCvMatrix()
{
    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(m_camera_matrix_file_path_, rows, cols, data_vec);

    if (rows != 3 || cols != 3)
    {
        std::cerr << "Intrinsic camera matrix has not shape 3x3, please check input file! " << std::endl;
        return cv::Mat::zeros(3, 3, CV_32FC1);
    }

    fillCvMat(m_cv_matrix_, data_vec, rows, cols);

    m_fx_ = data_vec[0];
    m_fy_ = data_vec[4];
    m_cx_ = data_vec[2];
    m_cy_ = data_vec[5];

    std::cout << "Intrinsic camera matrix loaded!" << std::endl;

    return m_cv_matrix_;
}
#endif

std::vector<std::vector<float>> CameraMatrix::getMat() const
{
    return m_matrix_;
}

#ifdef OpenCV_FOUND
cv::Mat CameraMatrix::getCvMat() const
{
    return m_cv_matrix_;
}
#endif

std::vector<std::vector<float>> IntrinsicMatrix::loadMatrix()
{
    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(m_camera_matrix_file_path_, rows, cols, data_vec);

    if (rows != 3 || cols != 3)
    {
        std::cerr << "Intrinsic camera matrix has not shape 3x3, please check input file! " << std::endl;
        return std::vector<std::vector<float>>{{0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
    }

    fillMat(m_matrix_, data_vec, rows, cols);

    m_fx_ = data_vec[0];
    m_fy_ = data_vec[4];
    m_cx_ = data_vec[2];
    m_cy_ = data_vec[5];

    return m_matrix_;
}

#ifdef OpenCV_FOUND
cv::Mat DistortionMatrix::loadCvMatrix()
{
    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(m_camera_matrix_file_path_, rows, cols, data_vec);

    if (rows != 1 || cols != 5)
    {
        std::cerr << "Distortion has not shape 1x5, please check input file! " << std::endl;
        return cv::Mat::zeros(1, 5, CV_32FC1);
    }

    fillCvMat(m_cv_matrix_, data_vec, rows, cols);

    m_k1_ = data_vec[0];
    m_k2_ = data_vec[1];
    m_p1_ = data_vec[2];
    m_p2_ = data_vec[3];
    m_k3_ = data_vec[4];

    std::cout << "Distortion matrix loaded!" << std::endl;
    return m_cv_matrix_;
}
#endif

std::vector<std::vector<float>> DistortionMatrix::loadMatrix()
{
    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(m_camera_matrix_file_path_, rows, cols, data_vec);

    if (rows != 1 || cols != 5)
    {
        std::cerr << "Distortion has not shape 1x5, please check input file! " << std::endl;
        return std::vector<std::vector<float>>{{0., 0., 0., 0., 0.}};
    }

    fillMat(m_matrix_, data_vec, rows, cols);

    m_k1_ = data_vec[0];
    m_k2_ = data_vec[1];
    m_p1_ = data_vec[2];
    m_p2_ = data_vec[3];
    m_k3_ = data_vec[4];

    std::cout << "Distortion matrix loaded!" << std::endl;
    return m_matrix_;
}

#ifdef OpenCV_FOUND
cv::Mat ProjectionMatrix::loadCvMatrix()
{
    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(m_camera_matrix_file_path_, rows, cols, data_vec);

    if (rows != 3 || cols != 4)
    {
        std::cerr << "Projection matrix has not shape 3x4, please check input file! " << std::endl;
        return cv::Mat::zeros(3, 4, CV_32FC1);
    }

    fillCvMat(m_cv_matrix_, data_vec, rows, cols);

    m_fx_ = data_vec[0];
    m_fy_ = data_vec[5];
    m_cx_ = data_vec[2];
    m_cy_ = data_vec[6];
    m_tx_ = data_vec[3];
    m_ty_ = data_vec[7];
    m_baseline_ = data_vec[3] / (-1. * data_vec[0]);

    std::cout << "Distortion matrix loaded!" << std::endl;
    return m_cv_matrix_;
}
#endif

std::vector<std::vector<float>> ProjectionMatrix::loadMatrix()
{
    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(m_camera_matrix_file_path_, rows, cols, data_vec);

    if (rows != 3 || cols != 4)
    {
        std::cerr << "Projection matrix has not shape 3x4, please check input file! " << std::endl;
        return std::vector<std::vector<float>>{{0., 0., 0., 0.}, {0., 0., 0., 0.}, {0., 0., 0., 0.}};
    }

    fillMat(m_matrix_, data_vec, rows, cols);

    m_fx_ = data_vec[0];
    m_fy_ = data_vec[5];
    m_cx_ = data_vec[2];
    m_cy_ = data_vec[6];
    m_tx_ = data_vec[3];
    m_ty_ = data_vec[7];
    m_baseline_ = data_vec[3] / (-1. * data_vec[0]);

    std::cout << "Distortion matrix loaded!" << std::endl;
    return m_matrix_;
}

#ifdef OpenCV_FOUND
cv::Mat RectificationMatrix::loadCvMatrix()
{
    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(m_camera_matrix_file_path_, rows, cols, data_vec);

    if (rows != 3 || cols != 3)
    {
        std::cerr << "Rectification matrix has not shape 3x3, please check input file! " << std::endl;
        return cv::Mat::zeros(3, 3, CV_32FC1);
    }

    fillCvMat(m_cv_matrix_, data_vec, rows, cols);

    m_r_11_ = data_vec[0];
    m_r_12_ = data_vec[1];
    m_r_13_ = data_vec[2];
    m_r_21_ = data_vec[3];
    m_r_22_ = data_vec[4];
    m_r_23_ = data_vec[5];
    m_r_31_ = data_vec[6];
    m_r_32_ = data_vec[7];
    m_r_33_ = data_vec[8];

    std::cout << "Rectification matrix loaded!" << std::endl;
    return m_cv_matrix_;
}
#endif

std::vector<std::vector<float>> RectificationMatrix::loadMatrix()
{
    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(m_camera_matrix_file_path_, rows, cols, data_vec);

    if (rows != 3 || cols != 3)
    {
        std::cerr << "Rectification matrix has not shape 3x3, please check input file! " << std::endl;
        return std::vector<std::vector<float>>{{0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
    }

    fillMat(m_matrix_, data_vec, rows, cols);

    m_r_11_ = data_vec[0];
    m_r_12_ = data_vec[1];
    m_r_13_ = data_vec[2];
    m_r_21_ = data_vec[3];
    m_r_22_ = data_vec[4];
    m_r_23_ = data_vec[5];
    m_r_31_ = data_vec[6];
    m_r_32_ = data_vec[7];
    m_r_33_ = data_vec[8];

    std::cout << "Rectification matrix loaded!" << std::endl;
    return m_matrix_;
}

#ifdef OpenCV_FOUND
cv::Mat ExtrinsicMatrix::loadCvMatrix()
{
    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(m_camera_matrix_file_path_, rows, cols, data_vec);

    if (rows != 3 || cols != 4)
    {
        std::cerr << "Extrinsic matrix has not shape 3x4, please check input file! " << std::endl;
        return cv::Mat::zeros(3, 3, CV_32FC1);
    }

    fillCvMat(m_cv_matrix_, data_vec, rows, cols);

    m_r_11_ = data_vec[0];
    m_r_12_ = data_vec[1];
    m_r_13_ = data_vec[2];
    m_tx_ = data_vec[3];
    m_r_21_ = data_vec[4];
    m_r_22_ = data_vec[5];
    m_r_23_ = data_vec[6];
    m_ty_ = data_vec[7];
    m_r_31_ = data_vec[8];
    m_r_32_ = data_vec[9];
    m_r_33_ = data_vec[10];
    m_tz_ = data_vec[11];

    std::cout << "Extrinsic matrix loaded!" << std::endl;
    return m_cv_matrix_;
}
#endif

std::vector<std::vector<float>> ExtrinsicMatrix::loadMatrix()
{
    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(m_camera_matrix_file_path_, rows, cols, data_vec);

    if (rows != 3 || cols != 4)
    {
        std::cerr << "Rectification matrix has not shape 3x3, please check input file! " << std::endl;
        return std::vector<std::vector<float>>{{0., 0., 0., 0.}, {0., 0., 0., 0.}, {0., 0., 0., 0.}};
    }

    fillMat(m_matrix_, data_vec, rows, cols);

    m_r_11_ = data_vec[0];
    m_r_12_ = data_vec[1];
    m_r_13_ = data_vec[2];
    m_tx_ = data_vec[3];
    m_r_21_ = data_vec[4];
    m_r_22_ = data_vec[5];
    m_r_23_ = data_vec[6];
    m_ty_ = data_vec[7];
    m_r_31_ = data_vec[8];
    m_r_32_ = data_vec[9];
    m_r_33_ = data_vec[10];
    m_tz_ = data_vec[11];

    std::cout << "Extrinsic matrix loaded!" << std::endl;
    return m_matrix_;
}

cv::Mat CalibrationData::getIntrinsicCvMatrix() const
{
    return m_intrinsic_matrix_.getCvMat();
}

cv::Mat CalibrationData::getExtrinsicCvMatrix() const
{
    return m_extrinsic_matrix_.getCvMat();
}

cv::Mat CalibrationData::getProjectionCvMatrix() const
{
    return m_projection_matrix_.getCvMat();
}

cv::Mat CalibrationData::getDistortionCvMatrix() const
{
    return m_distortion_matrix_.getCvMat();
}

cv::Mat CalibrationData::getRectificationCvMatrix() const
{
    return m_rectification_matrix_.getCvMat();
}

std::vector<std::vector<float>> CalibrationData::getIntrinsicMatrix() const
{
    return m_intrinsic_matrix_.getMat();
}

std::vector<std::vector<float>> CalibrationData::getExtrinsicMatrix() const
{
    return m_extrinsic_matrix_.getMat();
}

std::vector<std::vector<float>> CalibrationData::getProjectionMatrix() const
{
    return m_projection_matrix_.getMat();
}

std::vector<std::vector<float>> CalibrationData::getDistortionMatrix() const
{
    return m_distortion_matrix_.getMat();
}

std::vector<std::vector<float>> CalibrationData::getRectificationMatrix() const
{
    return m_rectification_matrix_.getMat();
}
