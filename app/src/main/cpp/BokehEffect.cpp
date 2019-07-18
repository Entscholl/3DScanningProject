#include "BokehEffect.h"
#include "StdHeader.h"
#include <memory>

struct COCListEntry{
	double depth;
	cv::Vec2i circleCenter;
};

void insertSorted(std::vector<COCListEntry> & list, COCListEntry l,const uint32_t maxSize){

	auto len = std::min((uint32_t)list.size(),maxSize);
	for(int i = 0; i < len;++i){
		COCListEntry & entry = list[i];
		if(entry.depth < l.depth){
			std::swap(entry,l);
		}
		len = std::min((uint32_t)list.size(),maxSize);
	}
	if(list.size() <maxSize){
		list.push_back(l);
	}
}

void BokehEffect::compute(){
	_outputImage = cv::Mat(_rgbInput.rows,_rgbInput.cols,CV_8UC3);
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

	_outputImage.setTo(cv::Scalar(0,0,0));
#pragma omp parallel for schedule(dynamic)
	for(int x = 0; x < _rgbInput.rows;++x) {
		for(int y = 0; y < _rgbInput.cols;++y) {
			auto depth = _depthInput.at<unsigned char>(x,y);
			auto inPixel = _rgbInput.at<cv::Vec3b>(x, y);
			if(depth >0) {
				double depthScale = (512 / depth) * _focalLength;
				unsigned int radius = std::ceil(2 * depthScale);
				for (int i = x - radius; i <= x + radius; ++i) {
					for (int j = y - radius; j <= y + radius; ++j) {
						const auto g = 1 / (2 * 3.1415 * depthScale) *
						               std::exp(-((i - x) * (i - x) + (j - y) * (j - y)) / (2 * depthScale));
						_outputImage.at<cv::Vec3b>(x, y) += g * inPixel;
					}
				}
				LOGI("x: %i, %i", x, radius);
			}
			else{
				_outputImage.at<cv::Vec3b>(x,y)=inPixel;
			}
		}

	}



}