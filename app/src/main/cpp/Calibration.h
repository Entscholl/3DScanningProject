
#ifndef INC_3DSCANNINGPROJECT_CALIBRATION_H
#define INC_3DSCANNINGPROJECT_CALIBRATION_H

#include <opencv2/core.hpp>
namespace StereoReconstruction {

    class Calibration {
    public:
        bool calibrate(const std::vector<cv::Mat>& calibration_images);
        cv::Mat get_camera_matrix();
        cv::Mat get_distortion_coefficients();

    private:
        cv::Mat camera_matrix;
        cv::Mat distortion_coefficients;
    };

}

#endif //INC_3DSCANNINGPROJECT_CALIBRATION_H
