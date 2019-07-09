#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

inline int square(int x) 
{ 
	return pow(x, 2);
}

int main() {

	int i, j;

	Mat left, right;

	//imread can't read png files, so please implement this
	//http://opencv-tutorials-hub.blogspot.com/2015/06/reading-images-sequentially-without-using-videocapture-displaying-using-for-loop-opencv-successive-frame-of-image-arjuntoshniwal.html

	//left = imread("C:\\Users\\iulia\\Documents\\TUM - MSc\\SS 2019\\3D\\3DScanningProject\\stereomatch\\StereoBlockMatching\\img\\1.jpg");
	//right = imread("C:\\Users\\iulia\\Documents\\TUM - MSc\\SS 2019\\3D\\3DScanningProject\\stereomatch\\StereoBlockMatching\\img\\1.jpg");


	//the disparity range defines how many pixels away from the block's location
    // in the first image to search for a matching block in the other image. I tried with this, not sure what we need for our photos.
	
	int disparityMIN = -50, disparityMAX = 50;
	int disparityRANGE = disparityMAX - disparityMIN;

	//the size of the blocks for block matching
    int halfBlockSize = 3;

	//int blockSize = 2 * halfBlockSize + 1;

	//get left image's size
	//int ROW = left.rows, COL = left.cols;

	int ROW = 50, COL = 50;

	// store the disparity and sum of squared distance (SSD)

	int** disp = new int* [ROW];
	for (int i = 0; i < ROW; ++i)
		disp[i] = new int[COL];

	int** SSD_value = new int* [ROW];
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

						SSD += square(left.at<cv::Vec3b>(left_row, left_col)[0] - right.at<cv::Vec3b>(right_row, min(max(0, right_col), COL - 1))[0])
							+ square(left.at<cv::Vec3b>(left_row, left_col)[1] - right.at<cv::Vec3b>(right_row, min(max(0, right_col), COL - 1))[1])
							+ square(left.at<cv::Vec3b>(left_row, left_col)[2] - right.at<cv::Vec3b>(right_row, min(max(0, right_col), COL - 1))[2]);
					}
				}

				if (SSD < SSD_value[i][j])
				{
					disp[i][j] = range;
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

	//imshow("disparity", dispIMG);
	//waitKey(0);

	return 0;
	
}