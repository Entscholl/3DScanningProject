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
    //cv::resize(A, A, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR_EXACT);
    //cv::resize(B, B, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR_EXACT);
    block_size = 10;
    num_disparities = 240;
    auto stereo_block_matcher = cv::StereoSGBM::create(0, num_disparities, block_size);
    //num disparities 240
    //block size 1
    stereo_block_matcher->setNumDisparities(num_disparities);
    stereo_block_matcher->setBlockSize(block_size);

    /*
    stereo_block_matcher->setP1(8*3*block_size*block_size);
    stereo_block_matcher->setP2(32*3*block_size*block_size);
    */
    /*
    stereo_block_matcher->setPreFilterCap(31);
    stereo_block_matcher->setUniquenessRatio(15);
    //stereo_block_matcher->setTextureThreshold(10);
    stereo_block_matcher->setSpeckleRange(32);
    stereo_block_matcher->setDisp12MaxDiff(1);
    stereo_block_matcher->setSpeckleWindowSize(100);
    stereo_block_matcher->setMinDisparity(0);
    */
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

void StereoReconstruction::StereoDepthPipeline::rectify_translation_estimate() {
    //I don't actually know why the transpose (= inverse because rotation) works here
    cv::Mat B_tmp;

    cv::warpPerspective(*inputB,B_tmp, cameraMatrixB*rotate*cameraMatrixB.inv(), inputB->size(),
            cv::INTER_CUBIC | CV_WARP_INVERSE_MAP);

    std::vector<cv::Point2f> points_A, points_B;

    get_matched_features(inputA, &B_tmp, points_A, points_B);
    //detect_corners(*inputA, A_corners, true);
    //detect_corners(B_tmp, B_corners, true);

    //match_keypoints(A_corners, B_corners, indices,nullptr, nullptr, inputA,&B_tmp );

    translate = estimate_translation(points_A, points_B, rotate.t(), cameraMatrixA,
            cameraMatrixB);
    LOGI("Estimated translation: %f, %f, %f", translate(0), translate(1), translate(2));
    //rectify();


    cv::Matx33f transform_x = cv::Matx33f::zeros();
    transform_x(0,1) = -translate(2);
    transform_x(0,2) = translate(1);

    transform_x(1,0) = translate(2);
    transform_x(1,2) = -translate(0);

    transform_x(2,0) = -translate(1);
    transform_x(2,1) = translate(0);

    cv::warpPerspective(*inputB,*inputB, cameraMatrixB*rotate*cameraMatrixB.inv(),
                        inputB->size(),cv::INTER_CUBIC | CV_WARP_INVERSE_MAP);

    for(auto point : points_A) {
        cv::circle(*inputA, point,10,cv::Scalar(0,255,0));
    }
    for(auto point : points_B) {
        cv::circle(*inputB, point,10,cv::Scalar(255,0,0));
    }
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
StereoReconstruction::StereoDepthPipeline::estimate_translation(const std::vector<cv::Point2f> &pointsA,
                                                                const std::vector<cv::Point2f> &pointsB,
                                                                const cv::Matx33f &rotation,
                                                                const cv::Matx33f &camera_matrix_A,
                                                                const cv::Matx33f &camera_matrix_B){
    cv::Vec2f result(0.0f);
    int counts = 0;
    cv::Matx33f F_tmp = rotation*camera_matrix_B.inv();
    for(int i = 0; i < pointsA.size();i++) {
        cv::Matx31f x1, x_1;
        x_1(0) = pointsA[i].x;
        x1(0) = pointsB[i].x;
        x_1(1) = pointsA[i].y;
        x1(1) = pointsB[i].y;
        x_1(2) = x1(2) = 1;
        x_1 = camera_matrix_A.inv()* x_1;
        x1 = -F_tmp*x1;
        float f =  x_1(2)*x1(1)-x_1(1)*x1(2);
        float d =  x_1(0)*x1(1)-x_1(1)*x1(0);
        f /= d;
        for(int j = i+1; j < pointsA.size(); j++) {
            cv::Matx31f x2, x_2;
            x_2(0) = pointsA[j].x;
            x2(0) = pointsB[j].x;
            x_2(1) = pointsA[j].y;
            x2(1) = pointsB[j].y;
            x_2(2) = x2(2) = 1;
            x_2 = camera_matrix_A.inv()* x_2;
            x2 = -F_tmp*x2;

            float t2 = x_2(2)*x2(1)-x_2(1)*x2(2)+(x_2(1)*x2(0)-x_2(0)*x2(1))*f;
            float n_ = x_1(0)*x1(2)-x_1(2)*x1(0);
            float d2 = n_/d;
            d2 *= x_2(0)*x2(1)-x_2(1)*x2(0);
            d2 += x_2(2)*x2(0) - x_2(0)* x2(2);
            t2 /= d2;
            float t3 = f + n_*t2/d;
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
    cv::BFMatcher matcher = cv::BFMatcher();
    //cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(
    //        cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    matcher.knnMatch(descriptors_A, descriptors_B, matches, 200);


    std::vector<int> indicesA, indicesB;
    double tresholdDist = 0.25*0.25 * double(inputA->size().height*inputA->size().height +
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

    cv::KeyPoint::convert(key_points_A, points_A, indicesA);
    cv::KeyPoint::convert(key_points_B, points_B, indicesB);
}
