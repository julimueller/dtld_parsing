#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <driveu_dataset.h>



int main(int argc, char** argv) {

    DriveuDatabase database;
    CalibrationData calib_left, calib_right;

    database.open("/media/muejul3/MyBook/DriveUDataset/Essen_all.yml");

    calib_left.loadIntrinsicMatrix("/home/muejul3/driveu-dataset-parsing/Calibration/intrinsic_left.yml");
    calib_left.loadProjectionMatrix("/home/muejul3/driveu-dataset-parsing/Calibration/projection_left.yml");
    calib_left.loadDistortionMatrix("/home/muejul3/driveu-dataset-parsing/Calibration/distortion_left.yml");
    calib_left.loadRectificationMatrix("/home/muejul3/driveu-dataset-parsing/Calibration/rectification_left.yml");
    calib_left.loadExtrinsicMatrix("/home/muejul3/driveu-dataset-parsing/Calibration/extrinsic.yml");
    calib_right.loadProjectionMatrix("/home/muejul3/driveu-dataset-parsing/Calibration/projection_right.yml");


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
        dispMat.copyTo(dispMat_viz);//dispMat / 255.0;
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

        cv::imshow("DriveU Dataset", im3);
        cv::waitKey(1);

        //std::cout << "Velocity: " << database.images[i].vehicle_data.velocity_ << " m/s, Yaw-Rate: " << database.images[i].vehicle_data.yaw_rate_ << " rad/s, Longitude: " << database.images[i].vehicle_data.longitude_ << " °, Latitude: " << database.images[i].vehicle_data.latitude_ << " °." << std::endl;



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
