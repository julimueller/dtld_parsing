#include <driveu_dataset.h>

/**
    @file       driveu_dataset.cpp
    @author     Julian Müller (julian.mu.mueller@daimler.com), Andreas Fregin (andreas.fregin@daimler.com)
    @date       v1: supports yml files  11/13/2017
                v2: support json fils   04/28/2020
    @brief      Contains class definitions and methods to parse the DriveU Database: Label format, load images, load disparity images and load calibration data. Either with or without OpenCV.
*/

/* ##################### DRIVEU IMAGE ##################### */

std::string DriveuImage::modifyImagePath(const std::string &data_base_dir, const std::string &file_path) const
{
    std::uint8_t cnt = 0;
    std::uint8_t i = 0;
    for (i = file_path.rfind("/"); i != std::string::npos; i = file_path.rfind("/", i - 1))
    {
        ++cnt;
        // Finding "cat" at the start means we're done.
        if (i == 0 || cnt == 4)
        {
            break;
        }
    }
    return data_base_dir + '/' + file_path.substr(i, file_path.size() - 1);
}

bool DriveuImage::parseImageDict(const Json::Value &image_dict, const std::string &data_base_dir)
{
    if (data_base_dir != "")
    {
        m_file_path_ = modifyImagePath(data_base_dir, image_dict["image_path"].asString());
        m_disp_file_path_ = modifyImagePath(data_base_dir, image_dict["disparity_image_path"].asString());
    }
    else
    {
        m_file_path_ = image_dict["image_path"].asString();
        m_disp_file_path_ = image_dict["disparity_image_path"].asString();
    }

    m_timestamp_ = image_dict["time_stamp"].asDouble();
    m_vehicle_data_.parseImageDict(image_dict);

    const Json::Value my_objects = image_dict["labels"];

    for (std::uint32_t index = 0; index < my_objects.size(); ++index)
    {
        DriveuObject label;
        label.parseObjectDict(my_objects[index]);
        m_objects_.push_back(label);
    }

    return true;
}

cv::Mat DriveuImage::getImage() const
{

    if (std::fstream(m_file_path_))
    {
        cv::Mat imageMat = cv::imread(m_file_path_, -1);
        cv::cvtColor(imageMat, imageMat, CV_BayerGB2BGR);
        imageMat.convertTo(imageMat, CV_MAKETYPE(CV_8U, imageMat.channels()), 1. / (1 << (12 - 8)));
        return imageMat;
    }
    else
    {
        std::cerr << "driveuDatabase:\tERROR: File " << m_file_path_ << " not found!" << std::endl;
        return cv::Mat();
    }
}

cv::Mat DriveuImage::getImage16Bit(Decompand &decompand)
{
    cv::Mat imageMat = cv::imread(m_file_path_, -1);

    cv::Mat linear;

    // decompand in place
    linear = imageMat.clone();
    unsigned short *dst = reinterpret_cast<unsigned short *>(linear.data);
    unsigned short *src = reinterpret_cast<unsigned short *>(imageMat.data);

    for (size_t p = 0; p < size_t(imageMat.rows) * size_t(imageMat.cols); ++p)
    {
        decompand.processPixel(src[p], dst[p]);
    }

    cv::cvtColor(linear, linear, CV_BayerGB2BGR);
    //linear.convertTo(linear, CV_MAKETYPE(CV_8U, linear.channels()), 1./(1<<(16-8)));

    return linear;
}

cv::Mat DriveuImage::getDisparityImage() const
{

    if (std::fstream(m_disp_file_path_))
    {

        cv::Mat imageMat = cv::imread(m_disp_file_path_, -1);
        cv::Mat dispMat = cv::Mat(imageMat.rows, imageMat.cols, CV_32F, cv::Scalar(0.));

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

        return dispMat;
    }
    else
    {
        std::cerr << "driveuDatabase:\tERROR: File " << m_disp_file_path_ << " not found!" << std::endl;
        return cv::Mat();
    }
}

/**
 * @brief DriveuImage::mapLabelsToDisparityImage() Returns all labels of the acutal image in disparity image coordinates. Original label coordinates are first rectified and then mapped via binning factors
 * @param calib Calibration of left camera
 * */
std::vector<cv::Rect> DriveuImage::mapLabelsToDisparityImage(CalibrationData &calib) const
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

void DriveuImage::visualizeDisparityImage(cv::Mat &dispMat) const
{

    double min, max;
    cv::minMaxIdx(dispMat, &min, &max);
    dispMat.convertTo(dispMat, CV_8UC1, 255 / (max - min), -min);
    cv::applyColorMap(dispMat, dispMat, cv::COLORMAP_JET);
}

cv::Mat DriveuImage::getLabeledImage() const
{
    cv::Mat imageMat = getImage();
    if (!imageMat.empty())
    {

        for (size_t i = 0; i < m_objects_.size(); ++i)
        {
            cv::rectangle(imageMat, m_objects_[i].getRect(), m_objects_[i].colorFromAttribute(), 2);
        }
        return imageMat;
    }

    return cv::Mat();
}

#endif

/* ##################### DRIVEU DATABASE ##################### */

bool DriveuDatabase::open(const std::string &path, const std::string &base_path)
{

    DriveuObject object;
    Json::Value root;
    Json::CharReaderBuilder builder;
    const std::string extension = path.substr(path.rfind('.') + 1);
    std::ifstream db;
    // check extension
    if (extension == "json")
    {
        db = std::ifstream(path, std::ifstream::binary);
    }
    // check v1 format
    else if (extension == "yml")
    {
        std::cerr << "Yaml files are deprecated. Either use the new .json files or use <git checkout v1.0> to jump back to yml support" << std::endl;
        return false;
    }
    else
    {
        std::cerr << "Label file with extension " << extension << " is not supported. Use .json instead!" << std::endl;
        return false;
    }
    std::string errs;
    bool ok = Json::parseFromStream(builder, db, &root, &errs);
    if (!ok)
    {
        // report to the user the failure and their locations in the document.
        std::cout << errs << "\n";
        return false;
    }

    const Json::Value my_images = root["images"];

    for (std::uint32_t index = 0; index < my_images.size(); ++index)
    {
        DriveuImage image;
        image.parseImageDict(my_images[index], base_path);
        m_images_.push_back(image);
    }
    return true;
}

/* ##################### DRIVEU OBJECT ##################### */

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

cv::Scalar DriveuObject::colorFromAttribute() const
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

cv::Rect DriveuObject::getRect() const
{
    return cv::Rect(m_x_, m_y_, m_width_, m_height_);
}
