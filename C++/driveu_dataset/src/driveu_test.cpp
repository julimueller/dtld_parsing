#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <driveu_dataset.h>

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
    CalibrationData calib_left, calib_right;

    database.open(label_file, data_base_dir);

    calib_left.loadIntrinsicMatrix(calib_dir + "/intrinsic_left.yml");
    calib_left.loadProjectionMatrix(calib_dir + "/projection_left.yml");
    calib_left.loadDistortionMatrix(calib_dir +  + "/distortion_left.yml");
    calib_left.loadRectificationMatrix(calib_dir + "/rectification_left.yml");
    calib_left.loadExtrinsicMatrix(calib_dir + "/extrinsic.yml");
    calib_right.loadProjectionMatrix(calib_dir + "/projection_right.yml");


    #ifdef OpenCV_FOUND
    cv::Mat intrinsic_left = calib_left.getCvIntrinsicMatrix();
    cv::Mat projection_left = calib_left.getCvProjectionMatrix();
    cv::Mat distortion_left = calib_left.getCvDistortionMatrix();
    cv::Mat rectification_left = calib_left.getCvRectificationMatrix();
    cv::Mat projection_right = calib_right.getCvProjectionMatrix();
    cv::Mat extrinsic = calib_left.getCvExtrinsicMatrix();

    std::cout << "Left projection matrix :" << projection_left << std::endl;
    std::cout << "Right projection matrix :" << projection_right << std::endl;
    std::cout << "Left intrinsic matrix: " << intrinsic_left << std::endl;
    std::cout << "Left distortion matrix: " << distortion_left << std::endl;
    std::cout << "Left rectification matrix: " << rectification_left << std::endl;
    std::cout << "Extrinsic matrix (rear axis -> camera): " << extrinsic << std::endl;


    for (size_t i = 0; i < database.images.size(); ++i) {

        cv::Mat imageMat, dispMat;
        if (!database.images[i].getLabeledImage(imageMat)) {
            continue;
        }
        if(!database.images[i].getDisparityImage(dispMat)) {
            continue;
        }
        cv::Mat dispMat_viz;
        dispMat.copyTo(dispMat_viz);
        database.images[i].visualizeDisparityImage(dispMat_viz);

        std::vector<cv::Rect> rects = database.images[i].mapLabelsToDisparityImage(calib_left);

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

    #ifndef OpenCV_FOUND
    std::vector<std::vector<float>> intrinsic, projection, distortion, rectification;
    intrinsic = calib_left.getIntrinsicMatrix();
    projection = calib_left.getProjectionMatrix();
    distortion = calib_left.getDistortionMatrix();
    rectification = calib_left.getRectificationMatrix();
    
    // Display matrices
    for (size_t i = 0; i < rectification.size(); ++i) {
        for (size_t j = 0; j < rectification[0].size(); ++j) {
            std::cout << rectification[i][j] << std::endl;
        }
    }
    #endif
    

}
