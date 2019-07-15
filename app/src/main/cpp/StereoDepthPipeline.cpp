#include <opencv2/calib3d/calib3d_c.h>
#include "StereoDepthPipeline.h"
#include "StdHeader.h"

StereoReconstruction::StereoDepthPipeline::StereoDepthPipeline() {
    rotate = cv::Matx33d::eye();
    translate = cv::Vec3d::all(0);
    distortionCoefficientsA = cv::Mat::ones(1, 5, CV_64F);
    distortionCoefficientsB = cv::Mat::ones(1, 5, CV_64F);
    num_disparities = 1024;
    block_size = 29;
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

    //num_disparities /= 2;
    //if(num_disparities%16)
    //    num_disparities += 16-(num_disparities%16);
    //cv::resize(*inputA, A, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR_EXACT);
    //cv::resize(*inputB, B, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR_EXACT);
    cv::cvtColor(*inputA, A, CV_BGR2GRAY);
    cv::cvtColor(*inputB, B, CV_BGR2GRAY);
    auto stereo_block_matcher = cv::StereoSGBM::create(0,num_disparities, 3);
    stereo_block_matcher->setP1(8*3*block_size*block_size);
    stereo_block_matcher->setP2(32*3*block_size*block_size);
    stereo_block_matcher->setNumDisparities(num_disparities);
    stereo_block_matcher->setBlockSize(block_size);
    stereo_block_matcher->setPreFilterCap(31);
    stereo_block_matcher->setUniquenessRatio(15);
    //stereo_block_matcher->setTextureThreshold(10);
    stereo_block_matcher->setSpeckleRange(32);
    stereo_block_matcher->setDisp12MaxDiff(1);
    stereo_block_matcher->setSpeckleWindowSize(100);
    stereo_block_matcher->setMinDisparity(0);

    stereo_block_matcher->compute(A, B, temp_result );

    temp_result.convertTo(*output, CV_8U);
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

    //cv::Mat R1, R2, P1, P2, Q;
    cv::Matx33d R1, R2;
    cv::Matx34d P1, P2;
    cv::Matx44d Q;

    cv::stereoRectify(cameraMatrixA, distortionCoefficientsA,
                      cameraMatrixB, distortionCoefficientsB,
                      inputA->size(), rotate, translate, R1, R2, P1, P2, Q,
                      0, 1, inputA->size());

    cv::Mat remap[2][2];

    initUndistortRectifyMap(cameraMatrixA, distortionCoefficientsA, R1, P1, inputA->size(), CV_16SC2,
                            remap[0][0], remap[0][1]);
    initUndistortRectifyMap(cameraMatrixB, distortionCoefficientsB, R2, P2, inputA->size(), CV_16SC2,
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

void StereoReconstruction::StereoDepthPipeline::set_translate_vector(const cv::Vec3d& translate) {
    this->translate = translate;
}

void StereoReconstruction::StereoDepthPipeline::set_block_size(int block_size) {
    this->block_size = block_size;
}

void StereoReconstruction::StereoDepthPipeline::set_num_disparities(int num_disparities) {
    this->num_disparities = num_disparities;
}

void StereoReconstruction::StereoDepthPipeline::rectify_uncalibrated() {

    cv::Mat A;
    cv::Mat B;
    cv::cvtColor(*inputA, A, CV_BGR2GRAY);
    cv::cvtColor(*inputB, B, CV_BGR2GRAY);

    std::vector<cv::KeyPoint> key_points_A;
    std::vector<cv::KeyPoint> key_points_B;
    cv::Mat descriptors_A;
    cv::Mat descriptors_B;
    cv::Ptr<cv::ORB> detector = cv::ORB::create();

    detector->detectAndCompute(A, cv::Mat(), key_points_A, descriptors_A);
    detector->detectAndCompute(B, cv::Mat(), key_points_B, descriptors_B);

    std::vector<cv::DMatch> matches;
    std::vector<std::vector<cv::DMatch>> temp_matches;
    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(
            cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    matcher.knnMatch(descriptors_A, descriptors_B, temp_matches, 2);

    std::vector<cv::Point2f> points_A;
    std::vector<cv::Point2f> points_B;
    std::vector<int> indices;
    // Ratio Test, see Lowe et al
    float ratio = 0.70f;
    while(indices.size() < 15) {
        indices.clear();
        matches.clear();
        for (const auto &match :temp_matches) {
            if (match.size() > 1) {
                if (match[0].distance < match[1].distance * ratio) {
                    matches.push_back(match[0]);
                    indices.push_back(match[0].queryIdx);
                }
            }
        }
        ratio += 0.05f;
    }

    cv::KeyPoint::convert(key_points_A, points_A, indices);
    cv::KeyPoint::convert(key_points_B, points_B, indices);

    cv::Mat F = cv::findFundamentalMat(points_A, points_A, CV_FM_RANSAC, 1, 0.999);
    std::vector<cv::Point3f> linesA;
    cv::computeCorrespondEpilines(points_A, 1, F, linesA);
    std::vector<cv::Point3f> linesB;
    cv::computeCorrespondEpilines(points_B, 2, F, linesB);
    for(auto point: points_A) {
        cv::circle(*inputA, point,5,cv::Scalar(0,255,0));
    }
    for(auto point: points_B) {
        cv::circle(*inputB, point,5,cv::Scalar(255,0,0));
    }
    for(auto line: linesB) {
        cv::Point2f pt0(0.f,0.f),pt1(inputA->cols, inputA->rows);
        pt0.y = -line.z/line.y;
        pt1.y = -(line.x*pt1.x+line.z)/line.y;
        cv::line(*inputA,pt0, pt1, cv::Scalar(0,0,255));
    }
    for(auto line: linesA) {
        cv::Point2f pt0(0.f,0.f), pt1(inputB->cols, inputB->rows );
        pt0.y = -line.z/line.y;
        pt1.y = -(line.x*pt1.x+line.z)/line.y;
        cv::line(*inputB,pt0, pt1, cv::Scalar(0,125,255));
    }
    //cv::drawMatches(*inputA, key_points_A, *inputB, key_points_B, matches, *output);

    cv::Mat H1, H2;
    try {
        cv::stereoRectifyUncalibrated(points_A, points_B, F, inputA->size(), H1, H2);
        cv::warpPerspective(*inputA, *inputA, H1, inputA->size());
        cv::warpPerspective(*inputB, *inputB, H2, inputB->size());
    } catch (std::exception e) {
        LOGE("Could not rectify uncalibrated");
    }


}

