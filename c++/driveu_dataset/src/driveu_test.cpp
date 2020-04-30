#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "driveu_dataset.h"
#include "driveu_calibration.h"
#include "compand.h"

bool parseArgs(int argc, char **argv, std::string &label_file, std::string &calib_dir, std::string &data_base_dir)
{

    label_file = "";
    calib_dir = "";
    data_base_dir = "";

    for (int i = 0; i < argc; i++)
    {

        if ((std::string(argv[i]) == "-label_file") && (i + 1 < argc))
        {
            label_file = std::stringstream(argv[i + 1]).str();
            continue;
        }

        if ((std::string(argv[i]) == "-calib_dir") && (i + 1 < argc))
        {
            calib_dir = std::stringstream(argv[i + 1]).str();
            continue;
        }

        if ((std::string(argv[i]) == "-data_base_dir") && (i + 1 < argc))
        {
            data_base_dir = std::stringstream(argv[i + 1]).str();
            continue;
        }
    }

    if (label_file == "" || calib_dir == "")
    {
        std::cout << std::endl;
        std::cout << "=======================================================================================" << std::endl;
        std::cout << "\t\t\t      DRIVEU TEST PARAMETERS" << std::endl;
        std::cout << "=======================================================================================" << std::endl;
        std::cout << std::endl;
        std::cout << "Label file and/or calibration file empty!" << std::endl;
        std::cout << "Usage: driveu_test -label_file <label_file_path.yml> -calib_dir <path_to_calib> -data_base_dir <dtld_dir>" << std::endl;
        std::cout << std::endl;
        std::cout << "=======================================================================================" << std::endl;
        std::cout << "=======================================================================================" << std::endl;
        std::cout << std::endl;
        return false;
    }
    else
    {
        std::cout << std::endl;
        std::cout << "=======================================================================================" << std::endl;
        std::cout << "\t\t\t      DRIVEU TEST PARAMETERS" << std::endl;
        std::cout << "=======================================================================================" << std::endl;
        std::cout << std::endl;

        std::cout << "Label File:\t\t" << label_file << std::endl;
        std::cout << "Calibration Dir:\t" << calib_dir << std::endl;
        std::cout << "Data Base Dir:\t\t" << data_base_dir << std::endl;
        std::cout << std::endl;
        std::cout << "=======================================================================================" << std::endl;
        std::cout << "=======================================================================================" << std::endl;
        std::cout << std::endl;
        return true;
    }
}

int main(int argc, char **argv)
{

    std::string label_file, calib_dir, data_base_dir;

    // parse arguments
    if (!parseArgs(argc, argv, label_file, calib_dir, data_base_dir))
    {
        return 0;
    }

    // load database
    DriveuDatabase database;
    if (!database.open(label_file, data_base_dir)) {
        return false;
    }

    // load calibration
    const std::string intrinsic_matrix_file_path = calib_dir + "/intrinsic_left.yml";
    const std::string extrinsic_matrix_file_path = calib_dir + "/extrinsic.yml";
    const std::string projection_matrix_file_path = calib_dir + "/projection_left.yml";
    const std::string distortion_matrix_file_path = calib_dir + +"/distortion_left.yml";
    const std::string rectification_matrix_file_path = calib_dir + "/rectification_left.yml";

    CalibrationData calib(intrinsic_matrix_file_path, extrinsic_matrix_file_path, projection_matrix_file_path, distortion_matrix_file_path, rectification_matrix_file_path);

#ifdef OpenCV_FOUND

    // get calibration matrices
    cv::Mat intrinsic_left = calib.getIntrinsicCvMatrix();
    cv::Mat projection_left = calib.getProjectionCvMatrix();
    cv::Mat distortion_left = calib.getDistortionCvMatrix();
    cv::Mat rectification_left = calib.getRectificationCvMatrix();
    cv::Mat extrinsic_left = calib.getExtrinsicCvMatrix();

    std::cout << "\nLeft projection matrix:\n"
              << projection_left << std::endl;
    std::cout << "\nLeft intrinsic matrix:\n"
              << intrinsic_left << std::endl;
    std::cout << "\nLeft distortion matrix:\n"
              << distortion_left << std::endl;
    std::cout << "\nLeft rectification matrix:\n"
              << rectification_left << std::endl;
    std::cout << "\nExtrinsic matrix (rear axis -> camera):\n"
              << extrinsic_left << std::endl;

    // load decompanding instance if desired to get images in 16 bit
    std::map<int, std::vector<int>> map;
    map[1023] = {1023, 1};
    map[2559] = {4095, 2};
    map[3455] = {32767, 32};
    map[3967] = {65535, 64};
    Decompand decompand(map);

    // show all images
    for (size_t i = 0; i < database.m_images_.size(); ++i)
    {

        cv::Mat imageMat = database.m_images_[i].getLabeledImage();
        cv::Mat imageMat16 = database.m_images_[i].getImage16Bit(decompand);

        cv::Mat dispMat = database.m_images_[i].getDisparityImage();

        cv::Mat dispMat_viz;
        dispMat.copyTo(dispMat_viz);
        database.m_images_[i].visualizeDisparityImage(dispMat_viz);

        std::vector<cv::Rect> rects = database.m_images_[i].mapLabelsToDisparityImage(calib);

        for (size_t i = 0; i < rects.size(); ++i)
        {
            cv::rectangle(dispMat_viz, rects[i], cv::Scalar(255, 255, 255), 2);
        }

        cv::resize(imageMat, imageMat, cv::Size(dispMat_viz.cols, dispMat_viz.rows));

        cv::Mat im3(imageMat.rows, imageMat.cols + dispMat_viz.cols, CV_8UC3);
        cv::Mat left(im3, cv::Rect(0, 0, imageMat.cols, imageMat.rows));
        imageMat.copyTo(left);
        cv::Mat right(im3, cv::Rect(imageMat.cols, 0, imageMat.cols, imageMat.rows));
        dispMat_viz.copyTo(right);

        cv::imshow("DTLD images", im3);
        cv::waitKey(1);
    }
#endif

    // Non-OpenCV Matrices.
    std::vector<std::vector<float>> intrinsic, projection, distortion, rectification, extrinsic;
    intrinsic = calib.getIntrinsicMatrix();
    projection = calib.getProjectionMatrix();
    distortion = calib.getDistortionMatrix();
    extrinsic = calib.getExtrinsicMatrix();
    rectification = calib.getRectificationMatrix();

    // Display matrices
    for (size_t i = 0; i < rectification.size(); ++i)
    {
        std::cout << "[ ";
        for (size_t j = 0; j < rectification[0].size(); ++j)
        {
            std::cout << rectification[i][j] << ", ";
        }
        std::cout << "]" << std::endl;
    }
}
