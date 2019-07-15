#include "Calibration.h"
#include "StdHeader.h"

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

bool StereoReconstruction::Calibration::calibrate(const std::vector<cv::Mat> &calibration_images) {
    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size patternsize(8,8);
    for(auto & image: calibration_images) {
        std::vector<cv::Point2f> corners;
        cv::Mat grayscale;
        cv::cvtColor(image, grayscale, cv::COLOR_BGR2GRAY);
        if(cv::findChessboardCorners(grayscale, patternsize,corners)) {
            cv::cornerSubPix(grayscale, corners, cv::Size(11, 11),
                    cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT
                            , 30, 0.1));
            image_points.emplace_back(std::move(corners));
        }
    }
    std::vector<std::vector<cv::Point3f>> calibration_corners(1);
    for(int x = 0; x < patternsize.width; x++) {
        for(int y = 0; y < patternsize.height; y++) {
            calibration_corners[0].emplace_back(x*30, y*30, 0);
        }
    }
    calibration_corners.resize(image_points.size(), calibration_corners[0]);
    distortion_coefficients = cv::Mat::zeros(8, 1, CV_64F);
    camera_matrix = cv::Mat::eye(3,3, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(calibration_corners, image_points, calibration_images[0].size(),
            camera_matrix,distortion_coefficients, rvecs, tvecs);
    LOGI("rms %f", rms);
    return true;
}

cv::Mat StereoReconstruction::Calibration::get_distortion_coefficients() {
    return distortion_coefficients;
}

cv::Mat StereoReconstruction::Calibration::get_camera_matrix() {
    return camera_matrix;
}
