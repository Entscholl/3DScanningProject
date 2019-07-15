#include "BokehEffect.h"
#include <memory>

struct COCListEntry{
		double depth;
		cv::Vec2i circleCenter;
};

void insertSorted(std::vector<COCListEntry> & list, COCListEntry l,const uint32_t maxSize){

	for(int i = 0; i < std::min((uint32_t)list.size(),maxSize);++i){
		if(list[i].depth < l.depth){
			std::swap(list[i],l);
		}
	}
	if(list.size() <maxSize){
		list.push_back(l);
	}
}

void BokehEffect::compute(){
	_outputImage = cv::Mat(_rgbInput.rows,_rgbInput.cols,CV_8UC3);
	//First: prepare the list of nearest circles of confusion
	std::vector<std::vector<std::vector<COCListEntry>>> cocList;
	for(int x = 0; x < _rgbInput.rows;++x) {
		for(int y = 0; y < _rgbInput.cols;++y){
			cv::Vec2i circleCenter = {x,y};

			const double circleOfConfusionDiameter = theta(circleCenter);
			const double depth = _depthInput.at<double>(x,y);

			const double r = circleOfConfusionDiameter/2.0;
			const uint32_t rr = std::ceil(r);

			for(int xx = x-rr;xx < x+rr;++xx){
				for(int yy = y-rr;yy < y+rr;++yy){
					cv::Vec2i point = {xx,yy};
					if(cv::norm(point-circleCenter)<r){
						insertSorted(cocList[xx][yy],{depth,{x,y}},_cocListSize);
					}
				}
			}
		}
	}

	for(int x = 0; x < _rgbInput.rows;++x) {
		for(int y = 0; y < _rgbInput.cols;++y) {
			const auto & circles = cocList[x][y];
			cv::Vec3f bokehPixel = {0,0,0};
			double weights = 0;
			for(const auto & c : circles){
				const cv::Vec2i p = {x,y};
                const double CoCDiamHat = thetahat(p);
                //alpha_c of q = 4 / (diameter hat of p) ^ 2
                int alpha_c = 4 / (CoCDiamHat * CoCDiamHat);
				const double w = alpha_c ;
				bokehPixel+=_rgbInput.at<cv::Vec3f>(c.circleCenter)*w;
				weights += w;
			}
			bokehPixel /= weights;

			_outputImage.at<cv::Vec3b>(x,y) = bokehPixel;
		}
	}

}