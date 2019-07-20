#include "BokehEffect.h"
#include "StdHeader.h"
#include <memory>
#include <algorithm>
#include <omp.h>

struct COCListEntry {
		double depth;
		cv::Vec2i circleCenter;
};

void insertSorted(std::vector<COCListEntry> &list, COCListEntry l, const uint32_t maxSize) {

	auto len = std::min((uint32_t) list.size(), maxSize);
	for (int i = 0; i < len; ++i) {
		COCListEntry &entry = list[i];
		if (entry.depth < l.depth) {
			std::swap(entry, l);
		}
		len = std::min((uint32_t) list.size(), maxSize);
	}
	if (list.size() < maxSize) {
		list.push_back(l);
	}
}

void BokehEffect::compute() {
	//First: prepare the list of nearest circles of confusion

	/*
	 * 	std::vector<std::vector<std::vector<COCListEntry>>> cocList;
	cocList.resize(_rgbInput.rows);
#pragma omp parallel for
	for(int i = 0; i < _rgbInput.rows;++i){
		cocList[i] = {};
		cocList[i].resize(_rgbInput.cols);
		for(int j = 0; j <_rgbInput.cols;++j){
			cocList[i][j] = {};
			cocList[i][j].resize(_cocListSize);
			for(int k = 0; k < _cocListSize;++k) {
				cocList[i][j][k] = {FP_INFINITE,{0,0}};
			}
		}
	}
#pragma omp parallel for
	for(int x = 0; x < _rgbInput.rows;++x) {
		for(int y = 0; y < _rgbInput.cols;++y){
			cv::Vec2i circleCenter = {x,y};

			const double circleOfConfusionDiameter = theta(circleCenter);
			const auto depth = _depthInput.at<unsigned char>(x,y)/255.f;
			if(depth >0 && depth < std::max(_rgbInput.cols,_rgbInput.rows)) {
				const double r = circleOfConfusionDiameter / 2.0;
				const uint32_t rr = std::ceil(r);

				for (int xx = x - rr; xx < x + rr; ++xx) {
					for (int yy = y - rr; yy < y + rr; ++yy) {
						if (xx >= 0 && yy >= 0 && xx < _rgbInput.rows && yy < _rgbInput.cols) {
							cv::Vec2i point = {xx, yy};
							cv::Vec2i diff = point - circleCenter;
							if (std::sqrt(diff(0) * diff(0) + diff(1) * diff(1)) < r) {
								std::vector<COCListEntry> &list = cocList[xx][yy];
								insertSorted(list, COCListEntry{depth, {x, y}}, _cocListSize);
							}
						}
					}
				}
			}
		}
	}
#pragma omp parallel for
	for(int x = 0; x < _rgbInput.rows;++x) {
		for(int y = 0; y < _rgbInput.cols;++y) {
			const auto & circles = cocList[x][y];
			cv::Vec3b bokehPixel = {0,0,0};
			double weights = 0;
			for(const auto & c : circles){
				const cv::Vec2i p = {x,y};
                const double CoCDiamHat = thetahat(p);
                //alpha_c of q = 4 / (diameter hat of p) ^ 2
                int alpha_c = 4 / (CoCDiamHat * CoCDiamHat);
				const double w = alpha_c ;
				bokehPixel+=_rgbInput.at<cv::Vec3b>(c.circleCenter)*w;
				weights += w;
			}
			bokehPixel /= weights;

			_outputImage.at<cv::Vec3b>(x,y) = bokehPixel;
		}
	}*/
	//GaussianBlurBackup


	cv::Mat outputImage = cv::Mat(_rgbInput.rows, _rgbInput.cols, CV_8UC3);
	LOGI("focal length: %f", _focalLength);
	double start = omp_get_wtime();
#pragma omp parallel for schedule(guided)
	for (int x = 0; x < _rgbInput.rows; ++x) {
		for (int y = 0; y < _rgbInput.cols; ++y) {
			auto depth = _depthInput.at<unsigned char>(x, y);

			const float depthScale = std::max(
					std::min(5.f + 5 * (-(float) depth * (float) _focalLength) / 128.0f, 5.f), 0.f);
			const float stddev = depthScale;
			const int radius = (int) std::floor(stddev * 3);
			cv::Vec3b outPixel = {0, 0, 0};
			//double first_term = (1.f / sqrt(2 * 3.1415f * stddev * stddev));
			if (radius < 1) {
				outputImage.at<cv::Vec3b>(x, y) = _rgbInput.at<cv::Vec3b>(x, y);
				continue;
			}
			float first_term = (1.f / sqrt(2 * 3.1415f * stddev * stddev));
			for (unsigned int i = std::max<unsigned int>(x - radius, 0);
			     i <= std::min<int>(x + radius, _rgbInput.rows - 1); ++i) {

				const auto inPixel = _rgbInput.at<cv::Vec3b>(i, y);
				//double g = first_term * std::exp(-(((i - x) * (i - x)) / (2 * stddev * stddev)));
				float g = first_term * std::exp(-(((i - x) * (i - x)) / (2 * stddev * stddev)));
				if (g > 1) {
					g = 1;
				}
				outPixel += g * inPixel;

			}
			outputImage.at<cv::Vec3b>(x, y) = outPixel;
		}
		//LOGI("x: %i", x);
	}
	_outputImage = cv::Mat(_rgbInput.rows, _rgbInput.cols, CV_8UC3);
#pragma omp parallel for schedule(guided)
	for (int x = 0; x < outputImage.rows; ++x) {
		for (int y = 0; y < outputImage.cols; ++y) {
			auto depth = _depthInput.at<unsigned char>(x, y);

			const float depthScale = std::max(
					std::min(5.f + 5 * (-(float) depth * (float) _focalLength) / 128.0f, 5.f), 0.f);
			const float stddev = depthScale;
			const int radius = (int) std::ceil(stddev * 3);
			if (radius < 1) {
				_outputImage.at<cv::Vec3b>(x, y) = _rgbInput.at<cv::Vec3b>(x, y);
				continue;
			}
			cv::Vec3b outPixel = {0, 0, 0};
			double first_term = (1.f / sqrt(2 * 3.1415f * stddev * stddev));
			for (unsigned int j = std::max<unsigned int>(y - radius, 0);
			     j <= std::min<int>(y + radius, _rgbInput.cols - 1); ++j) {

				const auto inPixel = outputImage.at<cv::Vec3b>(x, j);
				double g = first_term * std::exp(-(((j - y) * (j - y)) / (2 * stddev * stddev)));
				if (g > 1) {
					g = 1;
				}
				outPixel += g * inPixel;


				_outputImage.at<cv::Vec3b>(x, y) = outPixel;
			}
		}
		//LOGI("x: %i", x);
	}
	double end = omp_get_wtime();
	LOGI("Blur took: %fs", end - start);
}