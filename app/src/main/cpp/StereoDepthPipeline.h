#ifndef INC_3DSCANNINGPROJECT_STEREODEPTHPIPELINE_H
#define INC_3DSCANNINGPROJECT_STEREODEPTHPIPELINE_H

#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

namespace StereoReconstruction {
    class StereoDepthPipeline {
    public:
        StereoDepthPipeline();
        void set_input_A(cv::Mat *matrix_a);

        void set_camera_matrix_A(const cv::Mat& camera_matrix_a);

        void set_distortion_coefficients_A(const cv::Mat& distortion_coeffs);

        void set_input_B(cv::Mat *matrix_b);

        void set_camera_matrix_B(const cv::Mat& camera_matrix_a);

        void set_distortion_coefficients_B(const cv::Mat& distortion_coeffs);

        void set_translate_vector(const cv::Vec3f& translate);

        void set_rotation_matrix(const cv::Matx33f& rotate);

        void stereo_match(cv::Mat *output);

        void rectify();
        void rectify_uncalibrated();
        void rectify_translation_estimate();

        void set_num_disparities(int num_disparities);
        void set_block_size(int block_size);
    private:
        void get_matched_features(cv::Mat *image_A, cv::Mat *image_B,
                                std::vector<cv::Point2f> &points_A,
                                std::vector<cv::Point2f> &points_B);
        void detect_corners(cv::Mat &image,std::vector<cv::Point2f> &corners,bool sub_pix = true,
                int max_corners = 200);
        void match_keypoints(std::vector<cv::KeyPoint> &pointsA, std::vector<cv::KeyPoint> &pointsB,
                std::vector<int> &indices,cv::Mat *descriptors_A = nullptr,
                             cv::Mat *descriptors_B = nullptr,
                             cv::Mat *imageA = nullptr,
                             cv::Mat *imageB = nullptr
                );
        void match_keypoints(std::vector<cv::Point2f> &pointsA, std::vector<cv::Point2f> &pointsB,
                             std::vector<int> &indices,cv::Mat *descriptors_A = nullptr,
                             cv::Mat *descriptors_B = nullptr,
                             cv::Mat *imageA = nullptr,
                             cv::Mat *imageB = nullptr
        );
        cv::Vec3f estimate_translation(const std::vector<cv::Point2f> &pointsA,
                                       const std::vector<cv::Point2f> &pointsB,
                                       const  cv::Matx33f &rotation,
                                       const cv::Matx33f &camera_matrix_A,
                                       const cv::Matx33f &camera_matrix_B);
    public:
        static StereoDepthPipeline& instance() {
            static std::unique_ptr<StereoDepthPipeline> instance(new StereoDepthPipeline);
            return *instance;
        }
    private:
        cv::Mat *inputA = nullptr, *inputB = nullptr;
        cv::Matx33f cameraMatrixA, cameraMatrixB;
        cv::Mat distortionCoefficientsA, distortionCoefficientsB;
        cv::Matx33f rotate;
        cv::Vec3f translate;
        int num_disparities;
        int block_size;
    };
}
#endif //INC_3DSCANNINGPROJECT_STEREODEPTHPIPELINE_H
