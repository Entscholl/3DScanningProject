//
// Created by manuel on 16.07.2019.
//

#ifndef INC_3DSCANNINGPROJECT_STEREOBLOCKMATCHING_H
#define INC_3DSCANNINGPROJECT_STEREOBLOCKMATCHING_H

#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

namespace StereoRecons {
    class StereoDisparityPipeline {
    public:
        StereoDisparityPipeline();
        void set_input_A(cv::Mat *matrix_a);
        void set_input_B(cv::Mat *matrix_b);
        void block_match(cv::Mat *output, bool blur = false);
        void set_num_disparities(int num_disparities);
        void set_block_size(int block_size);
        void blur_disparity(cv::Mat *img, float std_dev);
    public:
        static StereoDisparityPipeline& instance() {
            static std::unique_ptr<StereoDisparityPipeline> instance(new StereoDisparityPipeline);
            return *instance;
        }
    private:
        cv::Mat *inputA = nullptr, *inputB = nullptr;
        int num_disparities;
        int block_size;
    };
}
#endif //INC_3DSCANNINGPROJECT_STEREOBLOCKMATCHING_H