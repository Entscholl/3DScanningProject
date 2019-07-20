//
// Created by manuel on 16.07.2019.
//

#include "StereoBlockMatching.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>

#include "StdHeader.h"

using namespace cv;
using namespace std;

inline int square(int x)
{
    return pow(x, 2);
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



    cv::Mat left, right;
    left = *inputA;
    right = *inputB;

    //check if image was read succesfully
    if (left.empty()) {
        cout << "Left image cannot be read" << std::endl;
    }
    if (right.empty()) {
        cout << "Right image cannot be read" << std::endl;
    }
    //imshow("window", right);
    //imshow("window2", left);
    //waitKey(0);


    float col = 300;
    float row = 200;

    resize(left, left, Size(col, row));
    resize(right, right, Size(col, row));


    //the disparity range defines how many pixels away from the block's location
    // in the first image to search for a matching block in the other image. I tried with this, not sure what we need for our photos.

    int disparityMIN = -num_disparities, disparityMAX = num_disparities;
    int disparityRANGE = disparityMAX - disparityMIN;

    //the size of the blocks for block matching
    int halfBlockSize = block_size/2;

    //int blockSize = 2 * halfBlockSize + 1;

    //get left image's size
    int ROW = left.rows;
    int COL = left.cols;

    //int ROW = 50, COL = 50;

    // store the disparity and sum of squared distance (SSD)

    int** disp = new int*[ROW];
    for (int i = 0; i < ROW; ++i)
        disp[i] = new int[COL];

    int** SSD_value = new int*[ROW];
    for (int i = 0; i < ROW; ++i)
        SSD_value[i] = new int[COL];

    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            disp[i][j] = 0;
            SSD_value[i][j] = 10e8;
        }
    }
    //block matching - with SSD calculation
    //from the left image, they go onto the corresponding parts in the right part
    //no epipolar lines ---> needs to be added

    int SSD = 0, left_row, left_col, right_row, right_col;

    for (int i = 0 + halfBlockSize; i < left.rows - halfBlockSize; i++)
    {
        for (int j = 0 + halfBlockSize; j < left.cols - halfBlockSize; j++)
        {

            for (int range = disparityMIN; range <= disparityMAX; range++)
            {
                SSD = 0;

                for (left_row = -halfBlockSize + i; left_row <= halfBlockSize + i; left_row++)
                {
                    for (left_col = -halfBlockSize + j; left_col <= halfBlockSize + j; left_col++)
                    {
                        right_row = left_row;
                        right_col = left_col + range;

                        SSD += square(left.at<cv::Vec3b>(left_row, left_col)[0] - right.at<cv::Vec3b>(right_row, min(max(0, right_col), COL - 1))[0]);
                        +square(left.at<cv::Vec3b>(left_row, left_col)[1] - right.at<cv::Vec3b>(right_row, min(max(0, right_col), COL - 1))[1])
                        + square(left.at<cv::Vec3b>(left_row, left_col)[2] - right.at<cv::Vec3b>(right_row, min(max(0, right_col), COL - 1))[2]);
                    }
                }

                if (SSD < SSD_value[i][j])
                {
                    disp[i][j] = range;
                    //cout << disp[i][j] << endl;
                    SSD_value[i][j] = SSD;
                }

            }

        }

    }

    //construct disparity image
    Mat dispIMG = Mat::zeros(ROW, COL, CV_8UC1);

    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            dispIMG.at<uchar>(i, j) = 63 + (int)192.0 * (disp[i][j] - disparityMIN) / disparityRANGE;
        }
    }
    float a = 1500;
    float b = 1000;
    resize(dispIMG, dispIMG, Size(a, b));

    imshow("disparity", dispIMG);
    waitKey(0);
    //imwrite("dispariy.jpg", dispIMG);
    //system("pause");


}
