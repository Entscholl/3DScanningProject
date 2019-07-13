#include "StereoDepthPipeline.h"
StereoReconstruction::StereoDepthPipeline::StereoDepthPipeline() {
    rotate = cv::Matx33d::eye();
    translate = cv::Matx13d::zeros();
    distortionCoefficientsA = cv::Mat::ones(1, 5, CV_64F);
    distortionCoefficientsB = cv::Mat::ones(1, 5, CV_64F);
    cameraMatrixA = cv::Matx33d::eye();
    cameraMatrixB = cv::Matx33d::eye();
}
void StereoReconstruction::StereoDepthPipeline::set_camera_matrix_A(const cv::Mat& camera_matrix) {
    cameraMatrixA = camera_matrix;
}

void StereoReconstruction::StereoDepthPipeline::set_distortion_coefficients_A(const cv::Mat&
                                                                              distortion_coeffs) {
    distortionCoefficientsA = distortion_coeffs;
}

void StereoReconstruction::StereoDepthPipeline::set_camera_matrix_B(const cv::Mat& camera_matrix) {
    cameraMatrixB = camera_matrix;
}

void StereoReconstruction::StereoDepthPipeline::set_distortion_coefficients_B(const cv::Mat&
                                                                              distortion_coeffs) {
    distortionCoefficientsB = distortion_coeffs;
}

void StereoReconstruction::StereoDepthPipeline::stereo_match(cv::Mat *output) {
    cv::Mat A;
    cv::Mat B;
    cv::Mat temp_result;

    // StereoBM wants one channel
    cv::cvtColor(*inputA, A, CV_BGR2GRAY);
    cv::cvtColor(*inputB, B, CV_BGR2GRAY);
    auto stereo = cv::StereoBM::create(256,7);
    stereo->compute(A, B, temp_result );

    //call Triangulation(???)
}

void StereoReconstruction::StereoDepthPipeline::rectify() {

//    cameraMatrix[0] = cv::initCameraMatrix2D({},{},inputA->size(),0);
//    cameraMatrix[1] = cv::initCameraMatrix2D({},{},inputA->size(),0);
//    cv::Mat R, T, E, F;
/*
double rms = stereoCalibrate({}, {}, {},
                             cameraMatrix[0], distCoeffs[0],
                             cameraMatrix[1], distCoeffs[1],
                             inputA->size(), R, T, E, F,
//                                 cv::CALIB_FIX_ASPECT_RATIO +
                             cv::CALIB_ZERO_TANGENT_DIST +
//                                 cv::CALIB_USE_INTRINSIC_GUESS +
                             cv::CALIB_SAME_FOCAL_LENGTH +
                             cv::CALIB_RATIONAL_MODEL +
                             cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
                             cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5) );
LOGI("Done with stereo calibration RMS: %f", rms);
*/

    cv::Mat R1, R2, P1, P2, Q;
    cv::Size new_size;

    cv::stereoRectify(cameraMatrixA, distortionCoefficientsA,
                      cameraMatrixB, distortionCoefficientsB,
                      inputA->size(), rotate, translate, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 1, new_size);

    cv::Mat remap[2][2];

    initUndistortRectifyMap(cameraMatrixA, distortionCoefficientsA, R1, P1, new_size, CV_16SC2,
                            remap[0][0], remap[0][1]);
    initUndistortRectifyMap(cameraMatrixB, distortionCoefficientsB, R2, P2, new_size, CV_16SC2,
                            remap[1][0], remap[1][1]);

    cv::Mat A_rectified;
    cv::Mat B_rectified;

    cv::remap(*inputA, *inputA, remap[0][0], remap[0][1], cv::INTER_LINEAR);
    cv::remap(*inputB, *inputB, remap[1][0], remap[1][1], cv::INTER_LINEAR);
    //call Rectify(Image A, delta Pose?);
    //call Rectify(Image B, delta Pose);
}

void StereoReconstruction::StereoDepthPipeline::set_input_A(cv::Mat *matrix_a) {
    inputA = matrix_a;
}

void StereoReconstruction::StereoDepthPipeline::set_input_B(cv::Mat *matrix_b) {
    inputB = matrix_b;
}

void StereoReconstruction::StereoDepthPipeline::set_rotation_matrix(const cv::Matx33d& rotate) {
    this->rotate = rotate;
}

void StereoReconstruction::StereoDepthPipeline::set_translate_vector(const cv::Matx13d& translate) {
    this->translate = translate;
}

