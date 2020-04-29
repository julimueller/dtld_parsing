#include <driveu_dataset.h>

/**
    @file       driveu_dataset.cpp
    @author     Julian Müller (julian.mu.mueller@daimler.com), Andreas Fregin (andreas.fregin@daimler.com)
    @date       v1: supports yml files  11/13/2017
                v2: support json fils   04/28/2020
    @brief      Contains class definitions and methods to parse the DriveU Database: Label format, load images, load disparity images and load calibration data. Either with or without OpenCV.
*/

/* ##################### DRIVEU OBJECT (LABEL) ##################### */

/**
 * @brief DriveuDatabase::open  Open the DriveU Database from file
 * @param m_x_         upper left corner
 * @param m_y_         upper left corner
 * @param width     width of label
 * @param height    height of label
 * @param unique_id Every TL has a unique ID
 * @param track_id  Track ID of the TL. Each instance of a traffic light has the same track id until the vehicle passed this instance. Vehicle passed the instance when the sequence changes. Detect sequence changes when sequence name (e.g. /2015-04-21_17-09-21/") in file path changes. Actually we use string notation, e.g. TrafficLight_1, TrafficLight_2 ...
*/



bool DriveuObject::parseObjectDict(const Json::Value &object_dict)
{

    // bounding box corner and size
    m_x_ = object_dict["x"].asInt();
    m_y_ = object_dict["y"].asInt();
    m_width_ = object_dict["w"].asInt();
    m_height_ = object_dict["h"].asInt();

    // track and unique identity
    m_track_id_ = object_dict["track_id"].asString();
    m_unique_id_ = object_dict["unique_id"].asInt();

    // attributes (deprecated: class_id)
    m_direction_ = DriveuAttribute("direction", object_dict["attributes"]["direction"].asString());
    m_relevance_ = DriveuAttribute("relevance", object_dict["attributes"]["relevance"].asString());
    m_occlusion_ = DriveuAttribute("occlusion", object_dict["attributes"]["occlusion"].asString());
    m_orientation_ = DriveuAttribute("orientation", object_dict["attributes"]["orientation"].asString());
    m_aspects_ = DriveuAttribute("aspects", object_dict["attributes"]["aspects"].asString());
    m_state_ = DriveuAttribute("state", object_dict["attributes"]["state"].asString());
    m_pictogram_ = DriveuAttribute("pictogram", object_dict["attributes"]["pictogram"].asString());

    return true;
}

/* ##################### DRIVEU IMAGE ##################### */

/**
 * @brief DriveuImage::DriveuImage Constructor
 */
DriveuImage::DriveuImage()
{

    m_file_path_ = "";
    m_disp_file_path_ = "";
    m_timestamp_ = 0.0;
    m_objects_.clear();
    std::map<int, std::vector<int>> map;
    map[1023] = {1023, 1};
    map[2559] = {4095, 2};
    map[3455] = {32767, 32};
    map[3967] = {65535, 64};
    m_decomp_ = new Decompand(map);
}

bool DriveuImage::parseImageDict(const Json::Value &image_dict)
{

    m_file_path_ = image_dict["image_path"].asString();
    m_disp_file_path_ = image_dict["disparity_image_path"].asString();
    m_timestamp_ = image_dict["time_stamp"].asDouble();
    m_vehicle_data_.parseImageDict(image_dict);

    const Json::Value my_objects = image_dict["labels"];

    for (uint index = 0; index < my_objects.size(); ++index)
    {
        DriveuObject label;
        label.parseObjectDict(my_objects[index]);
        m_objects_.push_back(label);
    }

    return true;
}

/* ##################### DRIVEU DATABASE ##################### */

/**
 * @brief DriveuDatabase::DriveuDatabase    Constructor
 */
DriveuDatabase::DriveuDatabase()
{
}

/**
 * @brief DriveuDatabase::open  Open the DriveU Database from file
 * @param                       Path to database (yml)
 */
bool DriveuDatabase::open(const std::string &path, const std::string &base_path)
{

    DriveuObject object;
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::ifstream db(path, std::ifstream::binary);
    std::string errs;
    bool ok = Json::parseFromStream(builder, db, &root, &errs);
    if (!ok)
    {
        // report to the user the failure and their locations in the document.
        std::cout << errs << "\n";
        return false;
    }

    const Json::Value my_images = root["images"];

    for (uint index = 0; index < my_images.size(); ++index)
    {
        DriveuImage image;
        image.parseImageDict(my_images[index]);
        m_images_.push_back(image);
    }
    return true;
}



/**
 * @brief DriveuObject::colorFromAttribute    Returns color from classID of one object
 * DIGIT 1 : 0 = not relevant, 1 = relevant TL, 4 = occluded
 * DIGIT 2 : 1 = horizontal TL, 2 = vertical TL, 3 = horizontal without frame, 4 = vertical without frame, 6 = horizontal Bus/Tram TL, 7 = vertical bus/tram, 8 = hor. without frame, 9 = vert. without frame
 * DIGIT 3 : 1 = single light TL, 2 = dual light TL, 3 = triple light TL (most common), ...
 * DIGIT 4 : 0 = light off, 1 = red, 2 = yellow, 3 = red-yellow, 4 = green, 5 = white, ...
 * DIGIT 5 : 1 = no light mask, 2 = arrow straight, 3 = arrow left + straight, 4 = arrow right, 5 = arrow right + straight, 7 = rttow left + right + straight, 8 = pedestrian mask in light, 9 = bike mask in light
 */
cv::Scalar DriveuObject::colorFromAttribute()
{

    if (m_state_.m_class_ == "red")
    {
        return cv::Scalar(0, 0, 255);
    }
    else if (m_state_.m_class_ == "yellow")
    {
        return cv::Scalar(0, 255, 255);
    }
    else if (m_state_.m_class_ == "red_yellow")
    {
        return cv::Scalar(0, 165, 255);
    }
    else if (m_state_.m_class_ == "green")
    {
        return cv::Scalar(0, 255, 0);
    }
    else
    {
        return cv::Scalar(255, 255, 255);
    }
}

/**
 * @brief DriveuObject::getRect()   Return object bounding box as cv Rect (upper left x, upper left y, width, height), x = horizontal axis, y = vertical axis
 * */
cv::Rect DriveuObject::getRect()
{
    return cv::Rect(m_x_, m_y_, m_width_, m_height_);
}

/**
 * @brief DriveuImage::getImage()  Return image in 8 Bit as BGR. Raw image is loaded from file, debayered and shifted from 12 bit to 8 bit
 * @param imageMat  Color image in 8 bit
 * */
bool DriveuImage::getImage(cv::Mat &imageMat)
{

    if (std::fstream(m_file_path_))
    {

        imageMat = cv::imread(m_file_path_, -1);
        cv::cvtColor(imageMat, imageMat, CV_BayerGB2BGR);
        imageMat.convertTo(imageMat, CV_MAKETYPE(CV_8U, imageMat.channels()), 1. / (1 << (12 - 8)));
        return true;
    }
    else
    {
        std::cerr << "driveuDatabase:\tERROR: File " << m_file_path_ << " not found!" << std::endl
                  << std::endl;
        return false;
    }
}

/**
 * @brief DriveuImage::getImage16Bit()  Return image in 16 Bit as BGR. Raw image is loaded from file and corrected with help of sensor characteristic
 * @param imageMat  Color image in 16 bit
 * */
cv::Mat DriveuImage::getImage16Bit()
{
    cv::Mat imageMat = cv::imread(m_file_path_, -1);

    cv::Mat linear;

    if (m_decomp_ == NULL)
    {
        linear = imageMat;
    }
    else
    {
        // decompand in place
        linear = imageMat.clone();
        unsigned short *dst = reinterpret_cast<unsigned short *>(linear.data);
        unsigned short *src = reinterpret_cast<unsigned short *>(imageMat.data);

        for (size_t p = 0; p < size_t(imageMat.rows) * size_t(imageMat.cols); ++p)
        {
            m_decomp_->processPixel(src[p], dst[p]);
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
bool DriveuImage::getDisparityImage(cv::Mat &dispMat)
{

    if (std::fstream(m_disp_file_path_))
    {

        cv::Mat imageMat = cv::imread(m_disp_file_path_, -1);
        dispMat = cv::Mat(imageMat.rows, imageMat.cols, CV_32F, cv::Scalar(0.));

        float scale_ = 1. / (1 << 4);
        const unsigned short *inImg = reinterpret_cast<const unsigned short *>(&imageMat.data[0]);

        float *outD = reinterpret_cast<float *>(&dispMat.data[0]);

        for (int i = 0; i < imageMat.rows * imageMat.cols; i++)
        {

            if (inImg[i] == 0xFFFF)
            {
                outD[i] = std::numeric_limits<float>::quiet_NaN();
            }
            else
            {
                // We have to "separate" disparity values from confidence values here
                outD[i] = (float)(((inImg[i]) & 0x0FFF) * (scale_));
            }
        }

        return true;
    }

    else
    {
        std::cerr << "driveuDatabase:\tERROR: File " << m_disp_file_path_ << " not found!" << std::endl
                  << std::endl;
        return false;
    }
}

/**
 * @brief DriveuImage::mapLabelsToDisparityImage() Returns all labels of the acutal image in disparity image coordinates. Original label coordinates are first rectified and then mapped via binning factors
 * @param calib Calibration of left camera
 * */
std::vector<cv::Rect> DriveuImage::mapLabelsToDisparityImage(CalibrationData &calib)
{

    std::vector<cv::Rect> rects_disparity;

    // binning factor: color image: 2048x1024, disparity image: 1024x440
    int binning_x = 2;
    int binning_y = 2;

    for (size_t i = 0; i < m_objects_.size(); ++i)
    {

        std::vector<cv::Point2f> distortedP, undistortedP;

        distortedP.push_back(cv::Point2f(m_objects_[i].m_x_, m_objects_[i].m_y_));
        distortedP.push_back(cv::Point2f(m_objects_[i].m_x_ + m_objects_[i].m_width_, m_objects_[i].m_y_ + m_objects_[i].m_height_));

        cv::undistortPoints(distortedP, undistortedP, calib.getIntrinsicCvMatrix(), calib.getDistortionCvMatrix(), calib.getRectificationCvMatrix(), calib.getProjectionCvMatrix());

        rects_disparity.push_back(cv::Rect(undistortedP[0].x / binning_x, undistortedP[0].y / binning_y, (undistortedP[1].x - undistortedP[0].x) / binning_x, (undistortedP[1].y - undistortedP[0].y) / binning_y));
    }

    return rects_disparity;
}


#ifdef OpenCV_FOUND

/**
 * @brief DriveuImage::visualizeDisparityImage() Modifies the disparity image and returns an CV_8UC3 image. Please note, that the pixel values cannot be used for 3D reconstruction anymore, use getDisparityImage() for this purpose!
 * */
void DriveuImage::visualizeDisparityImage(cv::Mat &dispMat)
{

    double min, max;
    cv::minMaxIdx(dispMat, &min, &max);
    dispMat.convertTo(dispMat, CV_8UC1, 255 / (max - min), -min);
    cv::applyColorMap(dispMat, dispMat, cv::COLORMAP_JET);
}

/**
 * @brief DriveuImage::getLabeledImage() Returns the actual image together with all labeled objects in this frame drawn as a bounding rectangle in the color of the actual TL state.
 * */
bool DriveuImage::getLabeledImage(cv::Mat &imageMat)
{

    if (getImage(imageMat))
    {

        for (size_t i = 0; i < m_objects_.size(); ++i)
        {
            cv::rectangle(imageMat, m_objects_[i].getRect(), m_objects_[i].colorFromAttribute(), 2);
        }
        return true;
    }

    else
    {
        return false;
    }
}

#endif
