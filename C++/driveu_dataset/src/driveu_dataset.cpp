#include <driveu_dataset.h>

/**
    @file       driveu_dataset.cpp
    @author     Julian Müller (julian-2.mueller@uni-ulm.de), Andreas Fregin (andreas.fregin@daimler.com)
    @date       11/13/2017
    @brief      Contains class definitions and methods to parse the DriveU Database: Label format, load images, load disparity images and load calibration data. Either with or without OpenCV.
*/

/* ##################### VEHICLE DATA ##################### */

/**
 * @brief VehicleData::VehicleData Constructor
 * @param latitude  GPS latitude
 * @param longitude GPS longitude
 * @param velocity  velocity in meters/second
 * @param yaw_rate  yaw rate in rad/s
 */
VehicleData::VehicleData() {

    this->latitude_ = 0.0;
    this->longitude_ = 0.0;
    this->velocity_ = 0.0;
    this->yaw_rate_ = 0.0;
}

/**
 * @brief VehicleData::~VehicleData() Destructor
 */
VehicleData::~VehicleData() {

}

/* ##################### DRIVEU OBJECT (LABEL) ##################### */

/**
 * @brief DriveuDatabase::open  Open the DriveU Database from file
 * @param x         upper left corner
 * @param y         upper left corner
 * @param width     width of label
 * @param height    height of label
 * @param class_id  class ID of the object
 *  * DIGIT 1 : 0 = not relevant, 1 = relevant TL, 4 = occluded
 * DIGIT 2 : 1 = horizontal TL, 2 = vertical TL, 3 = horizontal without frame, 4 = vertical without frame, 6 = horizontal Bus/Tram TL, 7 = vertical bus/tram, 8 = hor. without frame, 9 = vert. without frame
 * DIGIT 3 : 1 = single light TL, 2 = dual light TL, 3 = triple light TL (most common), ...
 * DIGIT 4 : 0 = light off, 1 = red, 2 = yellow, 3 = red-yellow, 4 = green, 5 = white, ...
 * DIGIT 5 : 1 = no light mask, 2 = arrow straight, 3 = arrow left + straight, 4 = arrow right, 5 = arrow right + straight, 7 = rttow left + right + straight, 8 = pedestrian mask in light, 9 = bike mask in light
 * @param unique_id Every TL has a unique ID
 * @param track_id  Track ID of the TL. Each instance of a traffic light has the same track id until the vehicle passed this instance. Vehicle passed the instance when the sequence changes. Detect sequence changes when sequence name (e.g. /2015-04-21_17-09-21/") in file path changes. Actually we use string notation, e.g. TrafficLight_1, TrafficLight_2 ...
*/
DriveuObject::DriveuObject() {

    this->x_ = 0;
    this->y_ = 0;
    this->width_ = 0;
    this->height_ = 0;
    this->class_id_ = 0;
    this->unique_id_ = 0;
    this->track_id_ = "";
}

/**
 * @brief DriveuObject::~DriveuObject() Destructor
 */
DriveuObject::~DriveuObject() {

}


/* ##################### DRIVEU IMAGE ##################### */

/**
 * @brief DriveuImage::DriveuImage Constructor
 */
DriveuImage::DriveuImage() {

    this->file_path_ = "";
    this->disp_file_path_ = "";
    this->timestamp_ = 0.0;
    this->objects.clear();
    std::map<int, std::vector<int>> map;
    map[1023] = {1023,1};
    map[2559] = {4095,2};
    map[3455] = {32767,32};
    map[3967] = {65535,64};
    this->decomp_ = new Decompand(map);
}

/**
 * @brief DriveuImage::~DriveuImage Destructor
 */
DriveuImage::~DriveuImage(){

}

/* ##################### DRIVEU DATABASE ##################### */

/**
 * @brief DriveuDatabase::DriveuDatabase    Constructor
 */
DriveuDatabase::DriveuDatabase(){

}

/**
 * @brief DriveuDatabase::~DriveuDatabase   Destructor
 */
DriveuDatabase::~DriveuDatabase() {

}

/**
 * @brief DriveuDatabase::open  Open the DriveU Database from file
 * @param                       Path to database (yml)
 */
bool DriveuDatabase::open(const std::string &path, const std::string &base_path) {

    DriveuObject object;
    DriveuImage image;

    std::string path_str = " path: ";
    std::string disp_path_str = "disp_path: ";
    std::string time_stamp_str = "time_stamp: ";
    std::string velocity_str = "velocity: ";
    std::string yaw_rate_str = "yaw_rate: ";
    std::string longitude_str = "longitude: ";
    std::string latitude_str = "latitude: ";
    std::ifstream inFile;
    std::string line = "initial";

    inFile.open(path.c_str());
    if (!inFile) {
        std::cout << "driveuDatabase:\tERROR: File "<< path << " not found!" << std::endl << std::endl;
        return false;
    }

    while (std::getline(inFile,  line)) {

        if (line[0] == '-') {

            std::getline(inFile,  line);

            while(line[2] == '-') {

                line.erase(0,7);
                std::string x, fill1, y, fill2, width, fill3, height, fill4, classid, fill5, unique_id, fill6, track_id;
                std::stringstream str(line);

                str >> x >> fill1 >> y >> fill2 >> width >> fill3 >> height >> fill4 >> classid >> fill5 >> unique_id >> fill6 >> track_id;

                std::stringstream(x.substr(0, x.size()-1)) >> object.x_;
                std::stringstream(y.substr(0, y.size()-1)) >> object.y_;
                std::stringstream(width.substr(0, width.size()-1))>> object.width_;
                std::stringstream(height.substr(0, height.size()-1)) >> object.height_ ;
                std::stringstream(classid.substr(0, classid.size()-1)) >> object.class_id_;
                std::stringstream(unique_id.substr(0, unique_id.size()-1)) >> object.unique_id_;
                std::stringstream(track_id.substr(0, track_id.size()-1)) >> object.track_id_;

                image.objects.push_back(object);

                std::getline(inFile,  line);

            }

        }

        if (line.find(path_str) != std::string::npos) {

            size_t pos = line.find(path_str);
            if (base_path.empty()) {
                image.file_path_ = line.substr(pos + path_str.size(), line.size() -1);
            }
            else {
                std::string file_path = line.substr(pos + path_str.size(), line.size() -1);
                size_t cnt = 0;
                int i = 0;
                for (i = file_path.rfind("/"); i != std::string::npos; i = file_path.rfind("/", i-1))
                {
                    ++cnt;
                    // Finding "cat" at the start means we're done.
                    if (i == 0 || cnt == 4) {
                       break;
                    }
                }
                image.file_path_ = base_path + '/' + file_path.substr(i, file_path.size()-1);
            }
        }

        else if (line.find(disp_path_str) != std::string::npos) {

            size_t pos = line.find(disp_path_str);
            image.disp_file_path_ = line.substr(pos + disp_path_str.size(), line.size() -1);
        }

        else if (line.find(time_stamp_str) != std::string::npos) {

            size_t pos = line.find(time_stamp_str);
            image.timestamp_ = std::stod(line.substr(pos + time_stamp_str.size(), line.size() -1));
        }

        else if (line.find(velocity_str) != std::string::npos) {

            size_t pos = line.find(velocity_str);
            image.vehicle_data.velocity_= std::stod(line.substr(pos + velocity_str.size(), line.size() -1));
        }

        else if (line.find(yaw_rate_str) != std::string::npos) {

            size_t pos = line.find(yaw_rate_str);
            image.vehicle_data.yaw_rate_ = std::stod(line.substr(pos + yaw_rate_str.size(), line.size() -1));
        }

        else if (line.find(longitude_str) != std::string::npos) {

            size_t pos = line.find(longitude_str);
            image.vehicle_data.longitude_ = std::stod(line.substr(pos + longitude_str.size(), line.size() -1));
        }

        else if (line.find(latitude_str) != std::string::npos) {

            size_t pos = line.find(latitude_str);
            image.vehicle_data.latitude_ = std::stod(line.substr(pos + latitude_str.size(), line.size() -1));
            images.push_back(image);
            image.objects.clear();

        }
    }

    return true;

}

/* ##################### CALIBRATION DATA ##################### */


/**
 * @brief CalibrationData::CalibrationData  Constructor
 */
CalibrationData::CalibrationData() {

}

/**
 * @brief CalibrationData::~CalibrationData  Destructor
 */
CalibrationData::~CalibrationData() {

}

/**
 * @brief CalibrationData::loadYmlMatrix()  Function loads a calibration matrix from .yml file format
 * @param path                              Path of YAML file
 * @param rows                              Returning number of rows of matrix
 * @param cols                              Returning number of cols of matrix
 * @param data_vec                          Returning matrix as vector of float
 */
bool CalibrationData::loadYmlMatrix(const std::string &path, int &rows, int &cols, std::vector<float> &data_vec) {

    std::ifstream inFile;

    std::string line = "initial";


    inFile.open(path.c_str());
    if (!inFile) {
        std::cout << "driveuDatabase:\tERROR: Intrinsic matrix file "<< path << " not found!" << std::endl << std::endl;
        return false;
    }

    std::string rows_str = "rows: ";
    std::string cols_str = "cols: ";
    std::string data_str = "data: ";

    while (std::getline(inFile,  line)) {

        if (line.find(rows_str) != std::string::npos) {
            size_t pos = line.find(rows_str);
            rows = std::stoi(line.substr(pos + rows_str.size(), line.size() -1));
        }

        else if (line.find(cols_str) != std::string::npos) {
            size_t pos = line.find(cols_str);
            cols = std::stoi(line.substr(pos + cols_str.size(), line.size() -1));
        }

        else if (line.find(data_str) != std::string::npos) {
            size_t pos = line.find(data_str);
            std::string data_all = line.substr(pos + data_str.size() +1 , line.size() - (pos + data_str.size() +1) -1);

            data_all.erase(std::remove(data_all.begin(), data_all.end(), ' '),data_all.end());
            std::stringstream ss(data_all);

            float i;

            while (ss >> i)
            {
                data_vec.push_back(i);

                if (ss.peek() == ',') {
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
bool CalibrationData::fillCvMat(cv::Mat &mat, std::vector<float> &vec, const int &rows, const int &cols) {

    mat  = cv::Mat(rows, cols, CV_32FC1);
    int idx = 0;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            mat.at<float>(i,j) = vec.at(idx);
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
bool CalibrationData::fillMat(std::vector<std::vector<float> > &mat, std::vector<float> &vec,  const int &rows, const int &cols) {

    int idx = 0;

    for (int i = 0; i < rows; ++i) {

        std::vector<float> temp;

        for (int j = 0; j < cols; ++j) {
            temp.push_back(float(vec.at(idx)));
            ++idx;
        }
        mat.push_back(temp);
    }

    return true;
}

/**
 * @brief CalibrationData::loadIntrinsicMatrix()    Method loading the intrinsic camera calibration (3x3 matrix) from path
*                                                       [fx  0 cx]
*                                                   K = [ 0 fy cy]
*                                                       [ 0  0  1]
 * @param path                                      Path of .YML file
 */
bool CalibrationData::loadIntrinsicMatrix(const std::string &path) {

    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(path, rows, cols, data_vec);

    if (rows != 3 || cols != 3) {
        std::cerr << "Intrinsic camera matrix has not shape 3x3, please check input file! " << std::endl;
        return false;
    }

    #ifdef OpenCV_FOUND
        fillCvMat(this->intrinsic_matrix.cv_intrinsic_matrix_, data_vec, rows, cols);
    #endif

    fillMat(this->intrinsic_matrix.intrinsic_matrix_, data_vec, rows, cols);

    this->intrinsic_matrix.fx_ = data_vec[0];
    this->intrinsic_matrix.fy_ = data_vec[4];
    this->intrinsic_matrix.cx_ = data_vec[2];
    this->intrinsic_matrix.cy_ = data_vec[5];

    std::cout << "Intrinsic camera matrix loaded!" << std::endl;
    return true;

}

/**
 * @brief CalibrationData::loadDistortionMatrix()   Method loading the intrinsic camera calibration (3x3 matrix) from path
*                                                   D = [ k1, k2, p1, p2, k3]
 * @param path                                      Path of .YML file
 */
bool CalibrationData::loadDistortionMatrix(const std::string &path) {

    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(path, rows, cols, data_vec);

    if (rows != 1 || cols != 5) {
        std::cerr << "Distortion has not shape 1x5, please check input file! " << std::endl;
        return false;
    }

    #ifdef OpenCV_FOUND
        fillCvMat(this->distortion_matrix.cv_distortion_matrix_, data_vec, rows, cols);
    #endif

    fillMat(this->distortion_matrix.distortion_matrix_, data_vec, rows, cols);

    this->distortion_matrix.k1_ = data_vec[0];
    this->distortion_matrix.k2_ = data_vec[1];
    this->distortion_matrix.p1_ = data_vec[2];
    this->distortion_matrix.p2_ = data_vec[3];
    this->distortion_matrix.k3_ = data_vec[4];

    std::cout << "Distortion matrix loaded!" << std::endl;
    return true;

}

/**
 * @brief CalibrationData::loadProjectionMatrix()   Method loading the projection matrix (3x4 matrix) from path
*                                                       [fx'  0  cx' Tx]
*                                                   P = [ 0  fy' cy' Ty]
*                                                       [ 0   0   1   0]
 * @param path                                      Path of .YML file
 */
bool CalibrationData::loadProjectionMatrix(const std::string &path) {

    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(path, rows, cols, data_vec);

    if (rows != 3 || cols != 4) {
        std::cerr << "Projection matrix has not shape 3x4, please check input file! " << std::endl;
        return false;
    }

    #ifdef OpenCV_FOUND
        fillCvMat(this->projection_matrix.cv_projection_matrix_, data_vec, rows, cols);
    #endif

    fillMat(this->projection_matrix.projection_matrix_, data_vec, rows, cols);

    this->projection_matrix.fx_ = data_vec[0];
    this->projection_matrix.fy_ = data_vec[5];
    this->projection_matrix.cx_ = data_vec[2];
    this->projection_matrix.cy_ = data_vec[6];
    this->projection_matrix.tx_ = data_vec[3];
    this->projection_matrix.ty_ = data_vec[7];
    this->projection_matrix.baseline_ = data_vec[3] /( -1. * data_vec[0]);


    std::cout << "Projection matrix loaded!" << std::endl;
    return true;

}

/**
 * @brief CalibrationData::loadRectificationMatrix  Method loading the rectification matrix (3x3 matrix) from path
*                                                       [r_11  r_12 r_13]
*                                                   R = [r_21  r_22 r_23]
*                                                       [r_31  r_32 r_33]
 * @param path                                      Path of .YML file
 */
bool CalibrationData::loadRectificationMatrix(const std::string &path) {

    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(path, rows, cols, data_vec);

    if (rows != 3 || cols != 3) {
        std::cerr << "Rectification matrix has not shape 3x4, please check input file! " << std::endl;
        return false;
    }

    #ifdef OpenCV_FOUND
        fillCvMat(this->rectification_matrix.cv_rectification_matrix_, data_vec, rows, cols);
    #endif

    fillMat(this->rectification_matrix.rectification_matrix_, data_vec, rows, cols);

    this->rectification_matrix.r_11_ = data_vec[0];
    this->rectification_matrix.r_12_ = data_vec[1];
    this->rectification_matrix.r_13_ = data_vec[2];
    this->rectification_matrix.r_21_ = data_vec[3];
    this->rectification_matrix.r_22_ = data_vec[4];
    this->rectification_matrix.r_23_ = data_vec[5];
    this->rectification_matrix.r_31_ = data_vec[6];
    this->rectification_matrix.r_32_ = data_vec[7];
    this->rectification_matrix.r_33_ = data_vec[8];

    std::cout << "Rectification matrix loaded!" << std::endl;
    return true;
}

/**
 * @brief CalibrationData::loadExtrinsicMatrix()    Method loading the extrinsic matrix (3x3 matrix) from path in ZYX order! It gives the translation + orientation from vehicle coordinate system (rear axis) to the left camera
*                                                       [r_11  r_12 r_13 t_x]
*                                                   E = [r_21  r_22 r_23 t_y]
*                                                       [r_31  r_32 r_33 t_z]
 * @param path                                      Path of .YML file
 */
bool CalibrationData::loadExtrinsicMatrix(const std::string &path) {

    int rows, cols;
    std::vector<float> data_vec;

    loadYmlMatrix(path, rows, cols, data_vec);

    if (rows != 3 || cols != 4) {
        std::cerr << "Extrinsic matrix has not shape 3x4, please check input file! " << std::endl;
        return false;
    }

    #ifdef OpenCV_FOUND
        fillCvMat(this->extrinsic_matrix.cv_extrinsic_matrix_, data_vec, rows, cols);
    #endif

    fillMat(this->extrinsic_matrix.extrinsic_matrix_, data_vec, rows, cols);

    this->extrinsic_matrix.r_11_ = data_vec[0];
    this->extrinsic_matrix.r_12_ = data_vec[1];
    this->extrinsic_matrix.r_13_ = data_vec[2];
    this->extrinsic_matrix.tx_ = data_vec[3];
    this->extrinsic_matrix.r_21_ = data_vec[4];
    this->extrinsic_matrix.r_22_ = data_vec[5];
    this->extrinsic_matrix.r_23_ = data_vec[6];
    this->extrinsic_matrix.ty_ = data_vec[7];
    this->extrinsic_matrix.r_31_ = data_vec[8];
    this->extrinsic_matrix.r_32_ = data_vec[9];
    this->extrinsic_matrix.r_33_ = data_vec[10];
    this->extrinsic_matrix.tz_ = data_vec[11];

    std::cout << "Extrinsic matrix loaded!" << std::endl;
    return true;
}

/**
 * @brief CalibrationData::getIntrinsicMatrix()    Method loading the intrinsic camera calibration (3x3 matrix) from path
*                                                       [fx  0 cx]
*                                                   K = [ 0 fy cy]
*                                                       [ 0  0  1]
* */
std::vector<std::vector<float>> CalibrationData::getIntrinsicMatrix() {
    return this->intrinsic_matrix.intrinsic_matrix_;
}

/**
 * @brief CalibrationData::getExtrinsicMatrix()    Method loading the extrinsic camera calibration (3x4 matrix) from path in ZYX order!  It gives the translation + orientation from vehicle coordinate system (rear axis) to the left camera
*                                                       [r_11  r_12 r_13 t_x]
*                                                   E = [r_21  r_22 r_23 t_y]
*                                                       [r_31  r_32 r_33 t_z]
* */
std::vector<std::vector<float>> CalibrationData::getExtrinsicMatrix() {
    return this->extrinsic_matrix.extrinsic_matrix_;
}

/**
 * @brief CalibrationData::getProjectionMatrix()   Method loading the projection matrix (3x4 matrix) from path
*                                                       [fx'  0  cx' Tx]
*                                                   P = [ 0  fy' cy' Ty]
*                                                       [ 0   0   1   0]
* */
std::vector<std::vector<float>> CalibrationData::getProjectionMatrix() {
    return this->projection_matrix.projection_matrix_;
}

/**
 * @brief CalibrationData::getDistortionMatrix()    Method loading the distortion matrix (1x5 matrix) from path
*                                                   D = [ k1, k2, p1, p2, k3]
* */
std::vector<std::vector<float>> CalibrationData::getDistortionMatrix() {
    return this->distortion_matrix.distortion_matrix_;
}

/**
 * @brief CalibrationData::getRectificationMatrix() Return Rectification Matrix in form
*                                                       [r_11  r_12 r_13]
*                                                   R = [r_21  r_22 r_23]
*                                                       [r_31  r_32 r_33]
 */
std::vector<std::vector<float>> CalibrationData::getRectificationMatrix() {
    return this->rectification_matrix.rectification_matrix_;
}

/* ##################### ONLY IF OPENCV FOUND ##################### */


#ifdef OpenCV_FOUND

/**
 * @brief DriveuObject::colorFromClassId    Returns color from classID of one object
 * DIGIT 1 : 0 = not relevant, 1 = relevant TL, 4 = occluded
 * DIGIT 2 : 1 = horizontal TL, 2 = vertical TL, 3 = horizontal without frame, 4 = vertical without frame, 6 = horizontal Bus/Tram TL, 7 = vertical bus/tram, 8 = hor. without frame, 9 = vert. without frame
 * DIGIT 3 : 1 = single light TL, 2 = dual light TL, 3 = triple light TL (most common), ...
 * DIGIT 4 : 0 = light off, 1 = red, 2 = yellow, 3 = red-yellow, 4 = green, 5 = white, ...
 * DIGIT 5 : 1 = no light mask, 2 = arrow straight, 3 = arrow left + straight, 4 = arrow right, 5 = arrow right + straight, 7 = rttow left + right + straight, 8 = pedestrian mask in light, 9 = bike mask in light
 */
cv::Scalar DriveuObject::colorFromClassId() {

    switch ((this->class_id_%100)/10) {
        case (1): return cv::Scalar(0,0,255);
        case (2): return cv::Scalar(0,255,255);
        case (3): return cv::Scalar(0,165,255);
        case (4): return cv::Scalar(0,255,0);
        default:   return cv::Scalar(255,255,255);
    }
}

/**
 * @brief DriveuObject::getRect()   Return object bounding box as cv Rect (upper left x, upper left y, width, height), x = horizontal axis, y = vertical axis
 * */
cv::Rect DriveuObject::getRect() {
    return cv::Rect(this->x_, this->y_, this->width_, this->height_);
}

/**
 * @brief DriveuImage::getImage()  Return image in 8 Bit as BGR. Raw image is loaded from file, debayered and shifted from 12 bit to 8 bit
 * @param imageMat  Color image in 8 bit
 * */
bool DriveuImage::getImage(cv::Mat &imageMat) {

    if (std::fstream(this->file_path_)) {

        imageMat = cv::imread(this->file_path_, -1);
        cv::cvtColor(imageMat, imageMat, CV_BayerGB2BGR);
        imageMat.convertTo(imageMat, CV_MAKETYPE(CV_8U, imageMat.channels()), 1./(1<<(12-8)));
        return true;

    }
    else {
        std::cerr << "driveuDatabase:\tERROR: File "<< this->file_path_ << " not found!" << std::endl << std::endl;
        return false;
    }


}

/**
 * @brief DriveuImage::getImage16Bit()  Return image in 16 Bit as BGR. Raw image is loaded from file and corrected with help of sensor characteristic
 * @param imageMat  Color image in 16 bit
 * */
cv::Mat DriveuImage::getImage16Bit() {
    cv::Mat imageMat = cv::imread(this->file_path_, -1);


    cv::Mat linear;

    if (decomp_ == NULL) {
        linear = imageMat;
    } else {
        // decompand in place
        linear = imageMat.clone();
        unsigned short* dst = reinterpret_cast<unsigned short*>(linear.data);
        unsigned short* src = reinterpret_cast<unsigned short*>(imageMat.data);

        for (size_t p = 0; p < size_t(imageMat.rows)*size_t(imageMat.cols); ++p) {
            decomp_->processPixel(src[p], dst[p]);
        }
    }

    cv::cvtColor(linear, linear, CV_BayerGB2BGR);
    //linear.convertTo(linear, CV_MAKETYPE(CV_8U, linear.channels()), 1./(1<<(16-8)));

    return linear;

}

/**
 * @brief DriveuImage::getDisparityImage() return disparity image as float, where each pixel contains the disparity in pixels. Please note, that the disparity image has a smaller resolution than the original image
 * and for the lower 144 pixels no disparity values are available
 * */
bool DriveuImage::getDisparityImage(cv::Mat &dispMat) {

    if (std::fstream(this->disp_file_path_)) {

        cv::Mat imageMat = cv::imread(this->disp_file_path_, -1);
        dispMat = cv::Mat(imageMat.rows, imageMat.cols, CV_32F, cv::Scalar(0.));

        float scale_ =  1./(1 << 4);
        const unsigned short* inImg =   reinterpret_cast<const unsigned short*>(&imageMat.data[0]);

        float* outD =                   reinterpret_cast<float*>(&dispMat.data[0]);

        for (int i = 0; i < imageMat.rows * imageMat.cols; i++) {

            if (inImg[i] == 0xFFFF) {
                outD[i] = std::numeric_limits<float>::quiet_NaN();

            }
            else {
                // We have to "separate" disparity values from confidence values here
                outD[i] = (float)(((inImg[i])&0x0FFF)*(scale_));
            }
        }

        return true;
    }

    else {
        std::cerr << "driveuDatabase:\tERROR: File "<< this->disp_file_path_ << " not found!" << std::endl << std::endl;
        return false;
    }

}

/**
 * @brief DriveuImage::mapLabelsToDisparityImage() Returns all labels of the acutal image in disparity image coordinates. Original label coordinates are first rectified and then mapped via binning factors
 * @param calib Calibration of left camera
 * */
std::vector<cv::Rect> DriveuImage::mapLabelsToDisparityImage(CalibrationData &calib) {

    std::vector<cv::Rect> rects_disparity;

    // binning factor: color image: 2048x1024, disparity image: 1024x440
    int binning_x = 2;
    int binning_y = 2;

    for (size_t i = 0; i < this->objects.size(); ++i) {

        std::vector<cv::Point2f> distortedP, undistortedP;

        distortedP.push_back(cv::Point2f(this->objects[i].x_, this->objects[i].y_));
        distortedP.push_back(cv::Point2f(this->objects[i].x_ + this->objects[i].width_ , this->objects[i].y_ + this->objects[i].height_));

        cv::undistortPoints(distortedP, undistortedP, calib.intrinsic_matrix.cv_intrinsic_matrix_, calib.distortion_matrix.cv_distortion_matrix_, calib.rectification_matrix.cv_rectification_matrix_, calib.projection_matrix.cv_projection_matrix_);

        rects_disparity.push_back(cv::Rect(undistortedP[0].x / binning_x, undistortedP[0].y / binning_y, (undistortedP[1].x - undistortedP[0].x) / binning_x, (undistortedP[1].y - undistortedP[0].y) / binning_y));
    }

    return rects_disparity;
}

/**
 * @brief DriveuImage::visualizeDisparityImage() Modifies the disparity image and returns an CV_8UC3 image. Please note, that the pixel values cannot be used for 3D reconstruction anymore, use getDisparityImage() for this purpose!
 * */
void  DriveuImage::visualizeDisparityImage(cv::Mat &dispMat) {

    double min, max;
    cv::minMaxIdx(dispMat, &min, &max);
    dispMat.convertTo(dispMat, CV_8UC1, 255 / (max-min), -min);
    cv::applyColorMap(dispMat, dispMat, cv::COLORMAP_JET);

}

/**
 * @brief DriveuImage::getLabeledImage() Returns the actual image together with all labeled objects in this frame drawn as a bounding rectangle in the color of the actual TL state.
 * */
bool DriveuImage::getLabeledImage(cv::Mat &imageMat) {

    if(getImage(imageMat)) {

        for (size_t i = 0; i < this->objects.size(); ++i) {
            cv::rectangle(imageMat, this->objects[i].getRect() , this->objects[i].colorFromClassId(), 2);
        }
        return true;
    }

    else {
        return false;
    }


}

/**
 * @brief CalibrationData::getCvIntrinsicMatrix()   Returns the intrinsic camera matrix as cv::Mat
 * */
cv::Mat CalibrationData::getCvIntrinsicMatrix() {
    return this->intrinsic_matrix.cv_intrinsic_matrix_;
}

/**
 * @brief CalibrationData::getCvIntrinsicMatrix()   Returns the extrinsic camera matrix as cv::Mat in ZYX order!
 * */
cv::Mat CalibrationData::getCvExtrinsicMatrix() {
    return this->extrinsic_matrix.cv_extrinsic_matrix_;
}

/**
 * @brief CalibrationData::getCvProjectionMatrix()  Returns the projection matrix as cv::Mat
 * */
cv::Mat CalibrationData::getCvProjectionMatrix() {
    return this->projection_matrix.cv_projection_matrix_;
}

/**
 * @brief CalibrationData::getCvDistortionMatrix() Returns the distortion matrix as cv::Mat
 * */
cv::Mat CalibrationData::getCvDistortionMatrix() {
    return this->distortion_matrix.cv_distortion_matrix_;
}

/**
 * @brief CalibrationData::getCvRectificationMatrix() Returns the rectification matrix as cv::Mat
 * */
cv::Mat CalibrationData::getCvRectificationMatrix() {
    return this->rectification_matrix.cv_rectification_matrix_;
}

#endif



