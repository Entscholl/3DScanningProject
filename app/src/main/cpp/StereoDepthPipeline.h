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

        void set_camera_matrix_A(cv::Mat camera_matrix_a);

        void set_distortion_coefficients_A(cv::Mat distortion_coeffs);

        void set_input_B(cv::Mat *matrix_b);

        void set_camera_matrix_B(cv::Mat camera_matrix_a);

        void set_distortion_coefficients_B(cv::Mat distortion_coeffs);

        void set_translate_vector(cv::Mat translate);

        void set_rotation_matrix(cv::Mat rotate);

        void stereo_match(cv::Mat *output);

        void rectify();

    public:
        static StereoDepthPipeline& instance() {
            static std::unique_ptr<StereoDepthPipeline> instance(new StereoDepthPipeline);
            return *instance;
        }
    private:
        cv::Mat *inputA = nullptr, *inputB = nullptr;
        cv::Mat cameraMatrixA, cameraMatrixB;
        cv::Mat distortionCoefficientsA, distortionCoefficientsB;
        cv::Mat rotate;
        cv::Mat translate;
    };
}
#endif //INC_3DSCANNINGPROJECT_STEREODEPTHPIPELINE_H
