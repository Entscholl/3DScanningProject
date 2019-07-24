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

void StereoRecons::StereoDisparityPipeline::block_match(cv::Mat *output){

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
    int halfBlockSize = block_size/2;

    //get left image's size
    const int width = A.size().width;
    const int height = A.size().height;

    // store the disparity and sum of squared distance (SSD)
    // Continous Memory Layout, might cause Cache improvements
    auto *disp = new short[height*width];
    //auto *SSD_value = new unsigned[height*width];

    //block matching - with SSD calculation
    //from the left image, they go onto the corresponding parts in the right part
    //no epipolar lines ---> needs to be added
    assert(inputA->isContinuous());
    assert(inputB->isContinuous());

    double start = omp_get_wtime();
#pragma omp parallel for schedule(guided)
    for (int i = 0 + halfBlockSize; i < height - halfBlockSize; i++)
    {
        for (int j = 0 + halfBlockSize; j < width - halfBlockSize; j++)
        {
            // These values are inside the loop, thus OMP will NOT try to
            // synchronize them which can cause serious performance implications
            unsigned prev_SSD = 0U - 1U;
            short prev_disp = 0;
            for (short range = static_cast<short>(disparityMIN); range <= disparityMAX; range++)
            {
                unsigned short SSD = 0;
                //uint16x8_t SSDs = vcreate_u16(0x0);
                for (int left_row = -halfBlockSize + i; left_row <= halfBlockSize + i; left_row++)
                {
                    for (int left_col = -halfBlockSize + j; left_col <= halfBlockSize + j; left_col++)
                    {

                        int right_row = left_row;
                        int right_col = std::clamp(left_col + range, 0, width - 1);

                        uint8_t *A_ptr = A.data+(left_row * width + left_col)*num_channels;
                        uint8_t *B_ptr = B.data+(right_row * width + right_col)*num_channels;
                        /*
                        uint8x8_t l_v = vld1q_dup_u8(A_ptr);
                        uint8x8_t r_v = vld1_u8(B_ptr);

                        uint8x8_t diff = vabd_u8(l_v, r_v);

                        SSDs = vmlal_u8(SSDs,diff, diff);
                        */
                        unsigned char l_[num_channels];
                        unsigned char r_[num_channels];
                        //Faster than .at, might be because Opencv is maybe not compiled in release
                        std::memcpy(l_, A_ptr, num_channels);
                        std::memcpy(r_, B_ptr, num_channels);
                        for(int c = 0; c < num_channels; c++)
                            SSD += square(((short)l_[c]) - r_[c]);
                        //Exit loop prematurely, did speedup by like 2x
                        if(SSD >= prev_SSD)  {
                            goto end;
                        }

                    }
                }

                if (SSD < prev_SSD)
                {
                    prev_disp = range;
                    prev_SSD = SSD;
                }end:;
            }
            //SSD_value[i*width+j] = prev_SSD;
            disp[i*width+j] = prev_disp;
        }
        //double end = omp_get_wtime();
        //LOGI("Stereo matching row (%d), thread (%d) took: %fs", i, omp_get_thread_num(), end - start);
    }
    double end = omp_get_wtime();
    LOGI("Stereo matching took: %fs", end - start);

    //construct disparity image
    //Mat dispIMG = Mat::zeros(ROW, COL, CV_8UC1);
    //cv::resize(*output, *output,inputA->size());
    cv::Mat temp_result(A.size(), CV_8UC1);
//#pragma omp parallel for schedule(guided)
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            temp_result.data[i*width + j] = static_cast<uchar>(63 + (int)192.0 * (disp[i*width+j] - disparityMIN) / disparityRANGE);
        }
    }

    temp_result.convertTo(*output, CV_8U);
    cv::resize(*output, *output, inputA->size());

    // Forgot the release memory
    delete[] disp;
    //delete[] SSD_value;
}
