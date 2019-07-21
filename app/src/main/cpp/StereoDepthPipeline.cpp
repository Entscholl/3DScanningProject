#include <opencv2/calib3d/calib3d_c.h>
#include <omp.h>
#include "StereoDepthPipeline.h"
#include "StereoBlockMatching.h"
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

void StereoReconstruction::StereoDepthPipeline::stereo_match(cv::Mat *output, bool blur) {
    cv::Mat A;
    cv::Mat B;
    cv::Mat temp_result;

    // StereoBM wants one channel

    cv::cvtColor(*inputA, A, CV_BGR2GRAY);
    cv::cvtColor(*inputB, B, CV_BGR2GRAY);
    //cv::resize(A, A, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR_EXACT);
    //cv::resize(B, B, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR_EXACT);

    //block_size = 8;
    //num_disparities = 128;
    //auto stereo_block_matcher = cv::StereoSGBM::create(0, num_disparities, block_size);
    //num disparities 240
    //block size 1
    auto stereo_block_matcher = cv::StereoSGBM::create(0,    //int minDisparity
                                                              240,     //int numDisparities
                                                              10);//bool fullDP = false
    stereo_block_matcher->compute(A, B, temp_result );

    if(blur)
        blur_disparity(&temp_result,2.f);

    //cv::resize(temp_result, temp_result, cv::Size(), 2, 2);
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
/*
    //cv::Mat R1, R2, P1, P2, Q;
    cv::Matx33f R1, R2;
    cv::Matx34f P1, P2;
    cv::Matx44f Q;

    cv::stereoRectify(cameraMatrixA, distortionCoefficientsA,
                      cameraMatrixB, distortionCoefficientsB,
                      inputA->size(), rotate.t(), translate, R1, R2, P1, P2, Q,
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
    */

}

void StereoReconstruction::StereoDepthPipeline::set_input_A(cv::Mat *matrix_a) {
    inputA = matrix_a;
}

void StereoReconstruction::StereoDepthPipeline::set_input_B(cv::Mat *matrix_b) {
    inputB = matrix_b;
}

void StereoReconstruction::StereoDepthPipeline::set_rotation_matrix(const cv::Matx33f& rotate) {
    this->rotate = rotate;
}

void StereoReconstruction::StereoDepthPipeline::set_translate_vector(const cv::Vec3f& translate) {
    this->translate = translate;
}

void StereoReconstruction::StereoDepthPipeline::set_block_size(int block_size) {
    this->block_size = block_size;
}

void StereoReconstruction::StereoDepthPipeline::set_num_disparities(int num_disparities) {
    this->num_disparities = num_disparities;
}

void StereoReconstruction::StereoDepthPipeline::rectify_uncalibrated(bool display_information, cv::Mat* output) {


    //cv::Mat B_tmp;

    //cv::warpPerspective(*inputB,B_tmp, cameraMatrixB*rotate*cameraMatrixB.inv(), inputB->size(),
    //                    cv::INTER_CUBIC | CV_WARP_INVERSE_MAP);

    std::vector<cv::Point2f> points_A;
    std::vector<cv::Point2f> points_B;

    get_matched_features(inputA, inputB, points_A, points_B);

    cv::Mat F = cv::findFundamentalMat(points_A, points_B, CV_FM_RANSAC, 1, 0.999);

    cv::Mat A_tmp, B_tmp;
    cv::Mat *A_ptr_tmp, *B_ptr_tmp;
    if(display_information) {
        if(output != nullptr) {
            A_tmp = inputA->clone();
            B_tmp = inputB->clone();
            A_ptr_tmp = inputA;
            B_ptr_tmp = inputB;
            inputA = &A_tmp;
            inputB = &B_tmp;
        }
        std::vector<cv::Point3f> linesA;
        cv::computeCorrespondEpilines(points_A, 1, F, linesA);
        std::vector<cv::Point3f> linesB;
        cv::computeCorrespondEpilines(points_B, 2, F, linesB);
        for (auto point: points_A) {
            cv::circle(*inputA, point, 5, cv::Scalar(0, 255, 0));
        }
        for (auto point: points_B) {
            cv::circle(*inputB, point, 5, cv::Scalar(255, 0, 0));
        }
        for (auto line: linesB) {
            cv::Point2f pt0(0.f, 0.f), pt1(inputA->cols, inputA->rows);
            pt0.y = -line.z / line.y;
            pt1.y = -(line.x * pt1.x + line.z) / line.y;
            cv::line(*inputA, pt0, pt1, cv::Scalar(0, 0, 255));
        }
        for (auto line: linesA) {
            cv::Point2f pt0(0.f, 0.f), pt1(inputB->cols, inputB->rows);
            pt0.y = -line.z / line.y;
            pt1.y = -(line.x * pt1.x + line.z) / line.y;
            cv::line(*inputB, pt0, pt1, cv::Scalar(0, 125, 255));
        }
    }
    //cv::drawMatches(*inputA, key_points_A, *inputB, key_points_B, matches, *output);

    cv::Mat H1, H2;
    try {
        cv::stereoRectifyUncalibrated(points_A, points_B, F, inputA->size(), H1, H2);
        H = H1;
        H_ = H2;
        cv::warpPerspective(*inputA, *inputA, H1, inputA->size());
        cv::warpPerspective(*inputB, *inputB, H2, inputB->size());
        if(output != nullptr) {
            cv::hconcat(*inputA, *inputB, *output);
        }
        if(display_information) {
            inputA  = A_ptr_tmp;
            inputB  = B_ptr_tmp;
            cv::warpPerspective(*inputA, *inputA, H1, inputA->size());
            cv::warpPerspective(*inputB, *inputB, H2, inputB->size());
        }
    } catch (std::exception e) {
        LOGE("Could not rectify uncalibrated");
    }


}

void StereoReconstruction::StereoDepthPipeline::rectify_translation_estimate(bool display_information, cv::Mat* output) {
    //I don't actually know why the transpose (= inverse because rotation) works here
    //cv::Mat B_tmp;

    //cv::warpPerspective(*inputB,B_tmp, cameraMatrixB*rotate*cameraMatrixB.inv(), inputB->size(),
    //        cv::INTER_CUBIC | CV_WARP_INVERSE_MAP);

    std::vector<cv::Point2f> points_A, points_B;

    get_matched_features(inputA, inputB, points_A, points_B);
    //detect_corners(*inputA, A_corners, true);
    //detect_corners(B_tmp, B_corners, true);

    //match_keypoints(A_corners, B_corners, indices,nullptr, nullptr, inputA,&B_tmp );

    translate = estimate_translation(points_A, points_B, rotate, cameraMatrixA, cameraMatrixB);
    //rectify();


    cv::Matx33f F = calculate_fundamental_matrix(rotate, cameraMatrixA, cameraMatrixB, translate);

    cv::Mat A_tmp, B_tmp;
    cv::Mat *A_ptr_tmp, *B_ptr_tmp;

    if(display_information) {
        A_tmp = inputA->clone();
        B_tmp = inputB->clone();
        A_ptr_tmp = inputA;
        B_ptr_tmp = inputB;
        inputA = &A_tmp;
        inputB = &B_tmp;

        std::vector<cv::Point3f> linesA;
        cv::computeCorrespondEpilines(points_A, 1, F, linesA);
        std::vector<cv::Point3f> linesB;
        cv::computeCorrespondEpilines(points_B, 2, F, linesB);
        for (auto point: points_A) {
            cv::circle(*inputA, point, 10, cv::Scalar(0, 255, 0));
        }
        for (auto point: points_B) {
            cv::circle(*inputB, point, 10, cv::Scalar(255, 0, 0));
        }
        for (auto line: linesB) {
            cv::Point2f pt0(0.f, 0.f), pt1(inputA->cols, inputA->rows);
            pt0.y = -line.z / line.y;
            pt1.y = -(line.x * pt1.x + line.z) / line.y;
            cv::line(*inputA, pt0, pt1, cv::Scalar(0, 0, 255));
        }
        for (auto line: linesA) {
            cv::Point2f pt0(0.f, 0.f), pt1(inputB->cols, inputB->rows);
            pt0.y = -line.z / line.y;
            pt1.y = -(line.x * pt1.x + line.z) / line.y;
            cv::line(*inputB, pt0, pt1, cv::Scalar(0, 125, 255));
        }
    }


    auto [H1, H2] = calculate_rectification_matrices(F);

    try {
        cv::Mat H1, H2;
        cv::stereoRectifyUncalibrated(points_A, points_B, F, inputA->size(), H1, H2);
        /*
        cv::Mat remap[2][2];
        cv::initUndistortRectifyMap(cameraMatrixA, std::vector<float>(),
                cameraMatrixA.inv()*H1*cameraMatrixA, cameraMatrixA, inputA->size(), CV_16SC2,
                remap[0][0], remap[0][1]);
        cv::initUndistortRectifyMap(cameraMatrixB, std::vector<float>(),
                cameraMatrixB.inv()*H2*cameraMatrixB, cameraMatrixB, inputB->size(), CV_16SC2,
                                    remap[1][0], remap[1][1]);
        cv::remap(*inputA, *inputA, remap[0][0], remap[0][1], CV_INTER_LINEAR);
        cv::remap(*inputB, *inputB, remap[1][0], remap[1][1], CV_INTER_LINEAR);
        */
        /*
        cv::Matx34d P = {0., (double)inputA->size().width, (double)inputA->size().width, 0,
                         0, 0, (double)inputA->size().height, (double)inputA->size().height,
                         1, 1,  1,  1};
        cv::Mat P_ = H1 * P;
        cv::Mat row_1 = P_.row(0) / P_.row(2);
        cv::Mat row_2 = P_.row(1) / P_.row(2);
        double minx, maxx, miny,maxy;
        cv::minMaxIdx(row_1, &minx, &maxx);
        cv::minMaxIdx(row_2, &miny, &maxy);
        LOGI("(%f,%f) (%f,%f)", minx, maxx,miny ,maxy );
        cv::Size new_size = cv::Size(maxx-minx, maxy-miny);
        cv::Matx33d mul = {1.,0,(double) std::min(-minx,0.),
                           0, 1, (double)  std::min(-miny,0.),
                           0, 0,  1};
        //H1 = H1*mul;
        LOGI("(%d,%d)", new_size.width, new_size.height);
        if(new_size.width > 4000 ||new_size.width <= 0 ||new_size.height <= 0  || new_size.height > 4000) new_size = inputA->size();
        cv::Mat tmp(new_size, CV_8UC3);
        */
        cv::Mat tmpA = inputA->clone();
        cv::Mat tmpB = inputB->clone();
        //cv::resize(*inputA, *inputA, new_size);
        //cv::resize(*inputB, *inputB, new_size);
        cv::warpPerspective(tmpA, *inputA, H1, inputA->size());
        //cv::warpPerspective(tmp, *inputB, H1, inputA->size(),  cv::WARP_INVERSE_MAP);
        cv::warpPerspective(tmpB, *inputB, H2, inputA->size());
        H = H1;
        H_ = H2;
        if(output != nullptr) {
            cv::hconcat(*inputA, *inputB, *output);
        }
        if(display_information) {
            inputA  = A_ptr_tmp;
            inputB  = B_ptr_tmp;
            cv::warpPerspective(*inputA, *inputA, H1, inputA->size());
            cv::warpPerspective(*inputB, *inputB, H2, inputB->size());
        }

    } catch (std::exception e) {
        LOGE("Could not rectify uncalibrated");
    }

    /*
    cv::warpPerspective(*inputB,*inputB, F,
                        inputB->size(),cv::INTER_CUBIC | CV_WARP_INVERSE_MAP);
    if(output != nullptr) {
        cv::hconcat(*inputA, *inputB, *output);
    }
    if(display_information) {
        inputA  = A_ptr_tmp;
        inputB  = B_ptr_tmp;
        cv::warpPerspective(*inputB,*inputB, F,
                            inputB->size(),cv::INTER_CUBIC | CV_WARP_INVERSE_MAP);
    }
    */
}

void StereoReconstruction::StereoDepthPipeline::detect_corners(cv::Mat &image,
                                                                 std::vector<cv::Point2f>  &corners,
                                                                 bool sub_pix,
                                                                 int max_corners) {
    cv::Mat tmp;
    cv::cvtColor(image, tmp, CV_BGR2GRAY);
    if(sub_pix) {
        double quality_level = 0.01;
        double min_distance = 10;
        int feature_block_size = 3;
        int gradient_size = 3;
        bool use_harris = false;
        double k = 0.04;
        cv::goodFeaturesToTrack(tmp, corners, max_corners, quality_level,
                min_distance, cv::Mat(), feature_block_size, gradient_size,
                use_harris, k);

        //cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
        //cv::dilate(dst, dst, kernel);
        //double min, max;
        //cv::minMaxLoc(dst, &min, &max);
        //cv::threshold(dst, dst, 0.01 * max, 255, CV_THRESH_BINARY);
        //cv::Mat labels, stats;
        //dst.convertTo(dst, CV_8U);
        //int n_labels = cv::connectedComponentsWithStats(dst, labels, stats, corners);
        cv::TermCriteria criteria(cv::TermCriteria::Type::MAX_ITER + cv::TermCriteria::EPS, 100,
                                  0.001);
        cv::cornerSubPix(tmp, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
    } else {
        cv::Mat dst = cv::Mat::zeros(tmp.size(), CV_32FC1);
        cv::cornerHarris(tmp, dst, 2, 3, 0.04);
        cv::Mat dst_norm;
        cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        for(int j = 0; j < dst_norm.rows;j++) {
            for(int i = 0; i < dst_norm.cols; i++) {
                if((int)dst_norm.at<float>(j,i) > 150){
                    corners.emplace_back(i,j);
                }
            }
        }
    }
}

void
StereoReconstruction::StereoDepthPipeline::match_keypoints(std::vector<cv::KeyPoint> &pointsA,
                                                           std::vector<cv::KeyPoint> &pointsB,
                                                           std::vector<int> &indices,
                                                           cv::Mat *descriptors_A,
                                                           cv::Mat *descriptors_B,
                                                           cv::Mat *imageA,
                                                           cv::Mat *imageB) {

    cv::Mat A_descriptors, B_descriptors;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor = cv::ORB::create();
    if(descriptors_A == nullptr) {
        if(imageA == nullptr) return;
        descriptorExtractor->compute(*imageA, pointsA, A_descriptors);
        descriptors_A = &A_descriptors;
    }
    if(descriptors_B == nullptr) {
        if(imageB == nullptr) return;
        descriptorExtractor->compute(*imageB, pointsB, B_descriptors);
        descriptors_B = &B_descriptors;
    }
    std::vector<std::vector<cv::DMatch>> matches;
    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(
            cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    matcher.knnMatch(*descriptors_A, *descriptors_B, matches, 2);

    for (const auto &match :matches) {

        if (match.size() > 1) {
            if (match[0].distance < match[1].distance * 0.6) {
                int id = match[0].queryIdx;
                indices.push_back(id);

            }
        }
    }
    LOGI("Found %d matches", indices.size());
}

void StereoReconstruction::StereoDepthPipeline::match_keypoints(std::vector<cv::Point2f> &pointsA,
                                                                std::vector<cv::Point2f> &pointsB,
                                                                std::vector<int> &indices,
                                                                cv::Mat *descriptors_A,
                                                                cv::Mat *descriptors_B,
                                                                cv::Mat *imageA, cv::Mat *imageB) {
    std::vector<cv::KeyPoint> A_keypoints, B_keypoints;
    cv::KeyPoint::convert(pointsA, A_keypoints);
    cv::KeyPoint::convert(pointsB, B_keypoints);
    match_keypoints(A_keypoints, B_keypoints, indices,descriptors_A, descriptors_B, imageA,imageB);
}

cv::Vec3f
StereoReconstruction::StereoDepthPipeline::estimate_translation(const std::vector<cv::Point2f> &X,
                                                                const std::vector<cv::Point2f> &X_,
                                                                const cv::Matx33f &rotation,
                                                                const cv::Matx33f &K,
                                                                const cv::Matx33f &K_){
    cv::Vec2f result(0.0f);
    int counts = 0;
    cv::Matx33f tmp = K*rotation.t();
    cv::Matx33f tmp2 = K * rotation * K_.inv();
    for(int i = 0; i < X.size();i++) {
        cv::Vec3f x1, x_1;
        x_1(0) = X[i].x;
        x1(0) = X_[i].x;
        x_1(1) = X[i].y;
        x1(1) = X_[i].y;
        x_1(2) = x1(2) = 1;
        cv::Matx13f a1 = -((tmp* x_1).cross(x1)).t() * tmp2;
        for(int j = i+1; j < X.size(); j++) {
            cv::Vec3f x2, x_2;
            x_2(0) = X[j].x;
            x2(0) = X_[j].x;
            x_2(1) = X[j].y;
            x2(1) = X_[j].y;
            x_2(2) = x1(2) = 1;
            // Solve a_1 dot t  = 0
            // and   a_2 dot t  = 0
            // Assuming t_x = 1
            cv::Matx13f a2 = -((tmp* x_2).cross(x2)).t() * tmp2;
            float t2 = a1(0)*a2(2)/a1(2)-a2(0);
            t2 /= (a2(1)-a1(0)*a2(2)/a1(2));
            float t3 = -(a1(0)+a1(1)*t2)/a1(2);
            //LOGI("candidate: (1.0, %f, %f)", t2, t3);
            if(!(isnan(t3) ||isnan(t2))) {
                result += cv::Vec2f(t2, t3);
                counts++;
            }
        }
    }
    //TODO other decision than mean
    if(counts != 0)
        result /= counts;
    cv::Vec3f t;

    t(0) = 1;
    t(1) = result(0);
    t(2) = result(1);
    //Assume |t| = 1;
    //t /= sqrt(t.dot(t));
    LOGI("Estimated translation: (%f, %f, %f)",t(0), t(1), t(2));
    return t;
}

void
StereoReconstruction::StereoDepthPipeline::get_matched_features(cv::Mat *image_A, cv::Mat *image_B,
                                                                std::vector<cv::Point2f> &points_A,
                                                                std::vector<cv::Point2f> &points_B) {
    cv::Mat A;
    cv::Mat B;
    cv::cvtColor(*image_A, A, CV_BGR2GRAY);
    cv::cvtColor(*image_B, B, CV_BGR2GRAY);

    std::vector<cv::KeyPoint> key_points_A;
    std::vector<cv::KeyPoint> key_points_B;
    cv::Mat descriptors_A;
    cv::Mat descriptors_B;
    cv::Ptr<cv::ORB> detector = cv::ORB::create();

    detector->detectAndCompute(A, cv::Mat(), key_points_A, descriptors_A);
    detector->detectAndCompute(B, cv::Mat(), key_points_B, descriptors_B);

    std::vector<std::vector<cv::DMatch>> matches;
    //cv::BFMatcher matcher = cv::BFMatcher();
    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(
            cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    matcher.knnMatch(descriptors_A, descriptors_B, matches, 2);



    std::vector<int> indicesA, indicesB;
    /*
    double tresholdDist = 0.20*0.20 * double(inputA->size().height*inputA->size().height +
                                             inputA->size().width*inputA->size().width);


    for (const auto &match :matches) {
        for(int i = 0; i < match.size(); i++){
            cv::Point2f from = key_points_A[match[i].queryIdx].pt;
            cv::Point2f to = key_points_B[match[i].trainIdx].pt;
            double x = from.x - to.x;
            x *= x;
            double y = from.y - to.y;
            y *= y;
            double norm = x + y;
            if (norm < tresholdDist && y < 25) {
                indicesA.push_back(match[i].queryIdx);
                indicesB.push_back(match[i].trainIdx);
            }
        }

    }
    */
    for (const auto &match :matches) {
        if (match.size() > 1) {
            if (match[0].distance < match[1].distance * 0.8) {
                indicesA.push_back(match[0].queryIdx);
                indicesB.push_back(match[0].trainIdx);
            }
        }
    }

    cv::KeyPoint::convert(key_points_A, points_A, indicesA);
    cv::KeyPoint::convert(key_points_B, points_B, indicesB);
}

cv::Matx33f
StereoReconstruction::StereoDepthPipeline::calculate_fundamental_matrix(const cv::Matx33f &R,
                                                                        const cv::Matx33f &K,
                                                                        const cv::Matx33f &K_,
                                                                        const cv::Vec3f &t) {


    //cv::Matx33f T = cv::Matx33f::zeros();
    // Cross product matrix [t]x x p = t x p
    cv::Matx33f T = { 0.f , -t(2), t(1),
                      t(2),  0.f ,-t(0),
                     -t(1),  t(0), 0.f};
     //Math courtesy of https://sourishghosh.com/2016/fundamental-matrix-from-camera-matrices/
    return K_.inv().t()*T*R*K.inv();
}

std::pair<cv::Matx33f, cv::Matx33f>
StereoReconstruction::StereoDepthPipeline::calculate_rectification_matrices(const cv::Matx33f &F) {
    cv::Mat S, U, V;
    cv::SVD::compute(F, S, U, V);

    log_float_mat(U, "U");
    log_float_mat(V, "V");
    cv::Matx31f e_= U.col(2);
    e_ *= 1.f/e_(2);
    cv::Matx13f e = V.row(2);
    e *= 1.f/e(2);
    /*
    e(2)= 1;
    e(1) = F(1,2)*F(0,2)/F(0,0) - F(1,2);
    e(1) /= (F(1,1)- F(1,0)*F(0,1)/F(0,0));
    e(0) = -F(0,1)*e(1)/F(0,0)-F(0,2)/F(0,0);

    cv::Matx33f F_ = F.t();
    e_(2)= 1;
    e_(1) = F_(1,2)*F_(2,2)/F_(2,0) - F_(1,2);
    e_(1) /= (F_(1,1)- F_(1,0)*F_(2,1)/F_(2,0));
    e_(0) = -F_(2,1)*e(1)/F_(2,0)-F_(2,2)/F_(2,0);

    */
    // e = (e_u, e_v, 1)^T
    LOGI("e: (%f, %f, %f)", e(0), e(1), e(2));
    // e' = (e'_u, e'_v, 1)^T
    LOGI("e': (%f, %f, %f)", e_(0), e_(1), e_(2));
    cv::Matx33f H = {1.F,       0,  0,
                     -e(1)/e(0), 1.F,  0,
                     -1.F/e(0),    0,  1.F};
    cv::Matx33f H_;
    cv::Matx33f F_ = { 0.F,0,  0,
                       0,  0, -1,
                       0,  1,  0};
    cv::solve(F.inv(),H.inv()*F_.inv(),H_,cv::DECOMP_SVD );
    //H_ = H_.t();

    return std::make_pair<cv::Matx33f, cv::Matx33f>(std::move(H), std::move(H_));
}

void StereoReconstruction::StereoDepthPipeline::log_float_mat(const cv::Mat &mat, const char* mat_name) {
    for(int i = 0; i < mat.rows; i++) {
        std::stringstream ss;
        for(int j = 0; j < mat.cols; j++){
            ss << mat.at<float>(i, j) << " ";
        }
        LOGI("%s: %s",mat_name, ss.str().c_str());
    }
}

void StereoReconstruction::StereoDepthPipeline::blur_disparity(cv::Mat *img, float std_dev) {
    std::vector<float> weights;
    const int radius = (int) std::floor(std_dev * 3);
    float first_term = (1.f / sqrt(2 * 3.1415f * std_dev * std_dev));
    for (int i = -radius; i <= radius; ++i) {
        float g = first_term * std::exp(-(((i +radius) * (i+radius)) / (2 * std_dev * std_dev)));
        weights.push_back(g);
    }

    cv::Mat depth_temp = cv::Mat(img->rows, img->cols, CV_16S);
    double start = omp_get_wtime();
    #pragma omp parallel for schedule(guided)
    for (int x = 0; x < img->rows; ++x) {
        for (int y = 0; y < img->cols; ++y) {
            short outPixel = 0;
            for (unsigned int i = std::max<unsigned int>(x - radius, 0);
                 i <= std::min<int>(x + radius, img->rows - 1); ++i) {
                const auto inPixel = img->at<short>(i, y);
                outPixel += inPixel * weights[i-x + radius];
            }
            depth_temp.at<short>(x, y) = outPixel;
        }
    }
    #pragma omp parallel for schedule(guided)
    for (int x = 0; x < img->rows; ++x) {
        for (int y = 0; y < img->cols; ++y) {
            short outPixel = 0;
            for (unsigned int j = std::max<unsigned int>(y - radius, 0);
                 j <= std::min<int>(y + radius, img->cols - 1); ++j) {
                const auto inPixel = depth_temp.at<short>(x, j);
                outPixel += inPixel * weights[j-y + radius];
            }
            img->at<short>(x, y) = outPixel;
        }
    }
    double end = omp_get_wtime();
    LOGI("Blur took: %fs", end - start);
}

void StereoReconstruction::StereoDepthPipeline::undo_rectification(cv::Mat *out) {
    cv::Mat tmp = out->clone();
    cv::warpPerspective(tmp, *out, H, out->size(), cv::WARP_INVERSE_MAP);
}

