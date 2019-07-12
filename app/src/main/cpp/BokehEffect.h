//
// Created by Michael on 12.07.2019.
//

#ifndef INC_3DSCANNINGPROJECT_BOKEHEFFECT_H
#define INC_3DSCANNINGPROJECT_BOKEHEFFECT_H

#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

class BokehEffect{
		cv::Mat & _rgbInput;
		cv::Mat & _depthInput;

		double _focalLength=0.2; // f in the paper
		double _aperture=0.01; // A in the paper

		double _dFocus = 1;

		unsigned int _cocListSize = 3;

		cv::Mat _outputImage;

public:
		BokehEffect(cv::Mat & inputImg,cv::Mat & depthImage)
			:_rgbInput(inputImg), _depthInput(depthImage)
		{

		}

		double & focalLenth(){return _focalLength;}
		double & aperture(){return _aperture;}
		double & dFocus(){ return _dFocus;}

		unsigned int cocListSize() {return _cocListSize;}

		double theta(const cv::Vec2i p){
			const auto depth = _depthInput.at<double>(p);
			return std::abs(_aperture*_focalLength*(_dFocus-depth)/(depth*(_dFocus -_focalLength)));
		}



		void compute();

		const cv::Mat & outputImage() const{
			return _outputImage;
		}
};

#endif //INC_3DSCANNINGPROJECT_BOKEHFILTER_H
