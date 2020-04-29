#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <driveu_dataset.h>
#include <driveu_calibration.h>

bool parseArgs(int argc, char **argv, std::string &label_file, std::string &calib_dir, std::string &data_base_dir) {

    label_file = "";
    calib_dir = "";
    data_base_dir = "";

    for (int i = 0; i < argc; i++) {

        if ((std::string(argv[i]) == "-label_file") && (i+1 < argc)) {
            label_file = std::stringstream(argv[i + 1]).str();
            continue;
        }

        if ((std::string(argv[i]) == "-calib_dir") && (i+1 < argc)) {
            calib_dir = std::stringstream(argv[i + 1]).str();
            continue;
        }

        if ((std::string(argv[i]) == "-data_base_dir") && (i+1 < argc)) {
            data_base_dir = std::stringstream(argv[i + 1]).str();
            continue;
        }
    }

    if (label_file == "" || calib_dir == "") {
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
    else {
        std::cout << std::endl;
        std::cout << "=======================================================================================" << std::endl;
        std::cout << "\t\t\t      DRIVEU TEST PARAMETERS" << std::endl;
        std::cout << "=======================================================================================" << std::endl;
        std::cout << std::endl;

        std::cout << "Label File:\t" << label_file << std::endl;
        std::cout << "Calibration Dir:\t\t" << calib_dir << std::endl;
        std::cout << "Data Dir:\t\t" << data_base_dir << std::endl;
        std::cout << std::endl;
        std::cout << "=======================================================================================" << std::endl;
        std::cout << "=======================================================================================" << std::endl;
        std::cout << std::endl;
        return true;
    }
}

int main(int argc, char** argv) {

    std::string label_file, calib_dir, data_base_dir;

    if (!parseArgs(argc, argv, label_file, calib_dir, data_base_dir)) {
        return 0;
    }

    DriveuDatabase database;
    database.open(label_file, data_base_dir);

    CalibrationData calib(calib_dir + "/intrinsic_left.yml", calib_dir + "/extrinsic.yml", calib_dir + "/projection_left.yml", calib_dir +  + "/distortion_left.yml", calib_dir + "/rectification_left.yml");

#ifdef OpenCV_FOUND

    cv::Mat intrinsic_left = calib.getIntrinsicCvMatrix();
    cv::Mat projection_left = calib.getProjectionCvMatrix();
    cv::Mat distortion_left = calib.getDistortionCvMatrix();
    cv::Mat rectification_left = calib.getRectificationCvMatrix();
    cv::Mat extrinsic_left = calib.getExtrinsicCvMatrix();

    std::cout << "Left projection matrix :" << projection_left << std::endl;
    std::cout << "Left intrinsic matrix: " << intrinsic_left << std::endl;
    std::cout << "Left distortion matrix: " << distortion_left << std::endl;
    std::cout << "Left rectification matrix: " << rectification_left << std::endl;
    std::cout << "Extrinsic matrix (rear axis -> camera): " << extrinsic_left << std::endl;

    std::cout << database.m_images_.size() << std::endl;
    for (size_t i = 0; i < database.m_images_.size(); ++i)
    {

        cv::Mat imageMat, dispMat;
        if (!database.m_images_[i].getLabeledImage(imageMat)) {
            continue;
        }
        if(!database.m_images_[i].getDisparityImage(dispMat)) {
            continue;
        }
        cv::Mat dispMat_viz;
        dispMat.copyTo(dispMat_viz);
        database.m_images_[i].visualizeDisparityImage(dispMat_viz);

        std::vector<cv::Rect> rects = database.m_images_[i].mapLabelsToDisparityImage(calib);

        for (size_t i = 0; i < rects.size(); ++i) {
            cv::rectangle(dispMat_viz, rects[i] , cv::Scalar(255,255,255), 2);
        }

        cv::resize(imageMat, imageMat, cv::Size(dispMat_viz.cols, dispMat_viz.rows));

        cv::Mat im3(imageMat.rows, imageMat.cols+ dispMat_viz.cols, CV_8UC3);
        cv::Mat left(im3, cv::Rect(0, 0, imageMat.cols, imageMat.rows));
        imageMat.copyTo(left);
        cv::Mat right(im3, cv::Rect(imageMat.cols, 0, imageMat.cols, imageMat.rows));
        dispMat_viz.copyTo(right);

        cv::imshow("DTLD images", im3);
        cv::waitKey(1);
    }
#endif

    std::vector<std::vector<float>> intrinsic, projection, distortion, rectification, extrinsic;
    intrinsic = calib.getIntrinsicMatrix();
    projection = calib.getProjectionMatrix();
    distortion = calib.getDistortionMatrix();
    extrinsic = calib.getExtrinsicMatrix();
    rectification = calib.getRectificationMatrix();

    // Display matrices
    for (size_t i = 0; i < rectification.size(); ++i) {
        std::cout << "[ ";
        for (size_t j = 0; j < rectification[0].size(); ++j)
        {
            std::cout << rectification[i][j] << ", ";
        }
        std::cout << "]" << std::endl;
    }


}
