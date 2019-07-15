#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>

using namespace cv;
using namespace std;

inline int square(int x) 
{ 
	return pow(x, 2);
}

int main() {

	int i, j;

	Mat left, right;

	left = cv::imread("im0e0.png");
	right = cv::imread("im1e0.png");
	
	//check if image was read succesfully
	if (!left.data) {
		cout << "Left image cannot be read" << std::endl;
	}
	if (!right.data) {
		cout << "Right image cannot be read" << std::endl;
	}
	//int max_disp = 32;
	//int wsize = 16;
	//Mat disparity_out;

	//compute disparity maps using opencv
	//Ptr<StereoSGBM> matcher = StereoSGBM::create(max_disp, wsize); 
	//matcher->compute(left, right, disparity_out);
	//imwrite("depth_test.png", disparity_out);
	
	cout << "Done!";

	//the disparity range defines how many pixels away from the block's location
    // in the first image to search for a matching block in the other image. I tried with this, not sure what we need for our photos.
	
	int disparityMIN = -50, disparityMAX = 50;
	int disparityRANGE = disparityMAX - disparityMIN;

	//the size of the blocks for block matching
     int halfBlockSize = 3;

	//int blockSize = 2 * halfBlockSize + 1;

	//get left image's size
	int ROW = left.rows; 
	int COL = left.cols;

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

	cout << "First Done!";
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
						cout << left_row << " " << left_col << "\n";

						right_row = left_row;
						right_col = left_col + range;

						cout << right_row << " " << right_col << "\n";

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
		cout << "Outer loop" << std::endl; 

	}
	cout << "Second Done!";

	//construct disparity image
	/*Mat dispIMG = Mat::zeros(ROW, COL, CV_8UC1); 

	for (int i = 0; i < ROW; i++)
	{
		for (int j = 0; j < COL; j++)
		{
			dispIMG.at<uchar>(i, j) = 63 + (int)192.0 * (disp[i][j] - disparityMIN) / disparityRANGE;
			cout << "innerloop2 Done";
		}
	}*/


	//imshow("disparity", dispIMG);
	//waitKey(0);

	return 0;
	
}
