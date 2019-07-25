//
// Created by manuel on 16.07.2019.
//

#include "StereoBlockMatching.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <omp.h>


#include "StdHeader.h"

using namespace cv;
using namespace std;

constexpr int square(int x)
{
    //Depending on the compiler pow could result in horrible performance
    return x*x;
}

StereoRecons::StereoDisparityPipeline::StereoDisparityPipeline() {
    num_disparities = 120;
    block_size = 4;
}
void StereoRecons::StereoDisparityPipeline::set_input_A(cv::Mat *matrix_a) {
    inputA = matrix_a;
}

void StereoRecons::StereoDisparityPipeline::set_input_B(cv::Mat *matrix_b) {
    inputB = matrix_b;
}
void StereoRecons::StereoDisparityPipeline::set_block_size(int block_size) {
    this->block_size = block_size;
}

void StereoRecons::StereoDisparityPipeline::set_num_disparities(int num_disparities) {
    this->num_disparities = num_disparities;
}

void StereoRecons::StereoDisparityPipeline::block_match(cv::Mat *output, bool blur){

    constexpr unsigned num_channels = 1;
    cv::Mat A(600, 900, CV_8UC3);
    cv::Mat B(600, 900, CV_8UC3);
    //Try grayscale instead of 3 channels for faster speeds
    //A = *inputA;
    //B = *inputB;
    cv::resize(*inputA, A, cv::Size(900, 600));
    cv::resize(*inputB, B, cv::Size(900, 600));
    cv::cvtColor(A, A, CV_BGR2GRAY);
    cv::cvtColor(B, B, CV_BGR2GRAY);
    //check if image was read succesfully
    if (A.empty()) {
        LOGE("Left Image empty");
        return;
    }
    if (A.channels() != num_channels) {
        LOGE("Left Image wrong amount of channels");
        return;
    }
    if (B.empty()) {
        LOGE("Right Image empty");
        return;
    }
    if (B.channels() != num_channels) {
        LOGE("Right Image wrong amount of channels");
        return;
    }



    //the disparity range defines how many pixels away from the block's location
    // in the first image to search for a matching block in the other image. I tried with this, not sure what we need for our photos.

    int disparityMIN = -num_disparities, disparityMAX = num_disparities;
    int disparityRANGE = disparityMAX - disparityMIN;

    //the size of the blocks for block matching
    //block_size = 9;
    const unsigned halfBlockSize = block_size/2;
    //const unsigned halfBlockSize = 8;
    //get left image's size
    const unsigned width = A.size().width;
    const unsigned height = A.size().height;
#define __ARM_NEON__
    #if defined(__ARM_NEON__)
    uint8x16_t temp_mask = vdupq_n_u8(0);
    switch(block_size) {
        case 16:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 15);
        case 15:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 14);
        case 14:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 13);
        case 13:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 12);
        case 12:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 11);
        case 11:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 10);
        case 10:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 9);
        case 9:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 8);
        case 8:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 7);
        case 7:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 6);
        case 6:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 5);
        case 5:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 4);
        case 4:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 3);
        case 3:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 2);
        case 2:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 1);
        case 1:
            temp_mask = vsetq_lane_u8(0xFF, temp_mask, 0);
        case 0:
            break;
        default:
            temp_mask = vcombine_u8(vcreate_u8(0x0000000000000000), vcreate_u8(0x0000000000000000));
            break;
    }
#endif
    assert(inputA->isContinuous());
    assert(inputB->isContinuous());

    cv::Mat temp_result(A.size(), CV_8UC1);
    //halfBlockSize = 8;
    double start = omp_get_wtime();
#pragma omp parallel for schedule(guided)// private(halfBlockSize)
    //block matching - with SSD calculation
    //from the left image, they go onto the corresponding parts in the right part
    //no epipolar lines ---> needs to be added

    for (int i = 0 + halfBlockSize; i < height - halfBlockSize; i++)
    {
#if defined(__ARM_NEON__)
        const unsigned border =  width - (2*8+1);

        const uint8x16_t mask = temp_mask;
#else
        const unsigned border =  width - (2*halfBlockSize+1);
#endif
        for (int j = 0 + halfBlockSize; j < width - halfBlockSize; j++)
        {
            // These values are inside the loop, thus OMP will NOT try to
            // synchronize them which can cause serious performance implications
            // Could solve that with OMP Clauses. Here, private(prev_SSD, SSD, ...)
            // But smaller variable scopes are actually preferred
            unsigned prev_SSD = 0U - 1U;
            short prev_disp = 0;
            for (short range = static_cast<short>(disparityMIN); range <= disparityMAX; range++)
            {
                unsigned short SSD = 0;
                //uint16x8_t SSDs = vcreate_u16(0x0);
                unsigned left_col = -halfBlockSize + j;
                unsigned right_col = std::clamp((unsigned)left_col + range, 0U, border);
                //const uint8_t *A_ptr = A.data+(left_row * width + left_col)*num_channels;
                //const uint8_t *B_ptr = B.data+(right_row * width + right_col)*num_channels;
                uint8_t *A_ptr = A.data+( (-halfBlockSize + i)*width + left_col)*num_channels;
                uint8_t *B_ptr = B.data+( (-halfBlockSize + i)*width + right_col)*num_channels;
                for (int left_row = -halfBlockSize + i; left_row <= halfBlockSize + i; left_row++)
                {
#if defined(__ARM_NEON__)
                    uint8x16_t l_v = vld1q_u8(A_ptr);
                    uint8x16_t r_v = vld1q_u8(B_ptr);
                    uint8x16_t diff = vabdq_u8(l_v, r_v);
                    diff = vandq_u8(mask, diff);
                    uint8x8_t low = vget_low_u8(diff);
                    uint8x8_t high = vget_high_u8(diff);
                    SSD += vaddvq_u16(vmull_u8(low, low));
                    SSD += vaddvq_u16(vmull_u8(high, high));

                    //Exit loop prematurely, did speedup by like 2x
                    if(SSD >= prev_SSD)  {
                        goto end;
                    }
                    A_ptr += width * num_channels;
                    B_ptr += width * num_channels;
#else
                   for (int left_col = -halfBlockSize + j; left_col <= halfBlockSize + j; left_col++) {
                        unsigned right_col = std::clamp((unsigned)left_col + range, 0U, border);
                        uint8_t *A_ptr = A.data+( (-halfBlockSize + i)*width + left_col)*num_channels;
                        uint8_t *B_ptr = B.data+( (-halfBlockSize + i)*width + right_col)*num_channels;
                        unsigned char l_[num_channels];
                        unsigned char r_[num_channels];
                        //Faster than .at, might be because Opencv is maybe not compiled in release
                        std::memcpy(l_, A_ptr, num_channels);
                        std::memcpy(r_, B_ptr, num_channels);
                        for(int c = 0; c < num_channels; c++)
                            SSD += square(((short)l_[c]) - r_[c]);
                        //Exit loop prematurely, did speedup by like 2x
                        if (SSD >= prev_SSD) {
                            goto end;
                        }
                    }

#endif

                }

                if (SSD < prev_SSD)
                {
                    prev_disp = range;
                    prev_SSD = SSD;
                }end:;
            }
            temp_result.data[i*width + j] = static_cast<uchar>((int)255.0 * (std::abs(prev_disp) - disparityMIN) / disparityRANGE);
        }
    }
    double end = omp_get_wtime();
    LOGI("Stereo matching took: %fs", end - start);

    temp_result.convertTo(*output, CV_8U);
    if(blur)
        blur_disparity(output, 3.f);
    cv::resize(*output, *output, inputA->size());

}
void StereoRecons::StereoDisparityPipeline::blur_disparity(cv::Mat *img, float std_dev) {
    std::vector<float> weights;
    const int radius = (int) std::floor(std_dev * 3);
    float first_term = (1.f / sqrt(2 * 3.1415f * std_dev * std_dev));
    float total = 0.f;
    for (int i = -radius; i <= radius; ++i) {
        float g = first_term * std::exp(-(((i +radius) * (i+radius)) / (2 * std_dev * std_dev)));
        total += g;
        weights.push_back(g);
    }
    for(auto &weight:weights)
        weight/= total;

    cv::Mat depth_temp = cv::Mat(img->rows, img->cols, CV_8U);
    double start = omp_get_wtime();
#pragma omp parallel for schedule(guided)
    for (int x = 0; x < img->rows; ++x) {
        for (int y = 0; y < img->cols; ++y) {
            unsigned char outPixel = 0;
            for (unsigned int i = std::max<unsigned int>(x - radius, 0);
                 i <= std::min<int>(x + radius, img->rows - 1); ++i) {
                const auto inPixel = img->at<unsigned char>(i, y);
                outPixel += inPixel * weights[i-x + radius];
            }
            depth_temp.at<unsigned char>(x, y) = outPixel;
        }
    }
#pragma omp parallel for schedule(guided)
    for (int x = 0; x < img->rows; ++x) {
        for (int y = 0; y < img->cols; ++y) {
            unsigned char outPixel = 0;
            for (unsigned int j = std::max<unsigned int>(y - radius, 0);
                 j <= std::min<int>(y + radius, img->cols - 1); ++j) {
                const auto inPixel = depth_temp.at<unsigned char>(x, j);
                outPixel += inPixel * weights[j-y + radius];
            }
            img->at<unsigned char>(x, y) = outPixel;
        }
    }
    double end = omp_get_wtime();
    LOGI("Blur took: %fs", end - start);
}
