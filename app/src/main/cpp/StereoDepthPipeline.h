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

        void set_translate_vector(const cv::Matx13d& translate);

        void set_rotation_matrix(const cv::Matx33d& rotate);

        void stereo_match(cv::Mat *output);

        void rectify();

    public:
        static StereoDepthPipeline& instance() {
            static std::unique_ptr<StereoDepthPipeline> instance(new StereoDepthPipeline);
            return *instance;
        }
    private:
        cv::Mat *inputA = nullptr, *inputB = nullptr;
        cv::Matx33d cameraMatrixA, cameraMatrixB;
        cv::Mat distortionCoefficientsA, distortionCoefficientsB;
        cv::Matx33d rotate;
        cv::Matx13d translate;
    };
}
#endif //INC_3DSCANNINGPROJECT_STEREODEPTHPIPELINE_H
