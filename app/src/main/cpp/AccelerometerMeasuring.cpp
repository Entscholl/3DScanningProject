
#include "AccelerometerMeasuring.h"

#include <stdlib.h>
#include <dlfcn.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/video/tracking.hpp>



int accelerometer_callback(int fd, int events, void *data) {
	ASensorEvent event;
	auto *_this = (AccelerometerMeasure *) data;
	if (ASensorEventQueue_getEvents(_this->_accelerometer_event_queue, &event, 1)) {
		if (event.type == ASENSOR_TYPE_ACCELEROMETER) {
			cv::Vec3d accVec{event.acceleration.x, event.acceleration.y, event.acceleration.z};
			if(accVec.dot(accVec) < 0.1){
				return 1;
			}
			if (_this->_startMeasuring) {
				_this->_accelerometer_measurements.push_back({std::move(accVec),
																									std::chrono::high_resolution_clock::now()});
			} else {
				if (_this->_accel_history.size() >= AccelerometerMeasure::HISTORY_SIZE) {
					_this->_accel_history.pop_front();
				}
				_this->_accel_history.emplace_back(std::move(accVec));
				if(_this->_timestamp_delta_t_valid) {
					double newDeltaT = (double) (event.timestamp - _this->_timestampeDelta_t)
					                   / (1000 * 1000 * 1000);
					_this->_kalman_delta_t = 0.8 * _this->_kalman_delta_t + 0.2 * newDeltaT;
				}
				else{
					_this->_timestamp_delta_t_valid = true;
				}
				_this->_timestampeDelta_t = event.timestamp;
			}
		}
	}
	return 1;
}

int gyroscope_callback(int fd, int event, void *data) {
	return 1;
}

ASensorManager *AcquireASensorManagerInstance() {
	typedef ASensorManager *(*PF_GETINSTANCEFORPACKAGE)(const char *name);
	void *androidHandle = dlopen("libandroid.so", RTLD_NOW);
	PF_GETINSTANCEFORPACKAGE getInstanceForPackageFunc = (PF_GETINSTANCEFORPACKAGE)
			dlsym(androidHandle, "ASensorManager_getInstanceForPackage");
	if (getInstanceForPackageFunc) {
		return getInstanceForPackageFunc(PACKAGE_NAME);
	}

	typedef ASensorManager *(*PF_GETINSTANCE)();
	PF_GETINSTANCE getInstanceFunc = (PF_GETINSTANCE)
			dlsym(androidHandle, "ASensorManager_getInstance");
	// by all means at this point, ASensorManager_getInstance should be available
	assert(getInstanceFunc);
	return getInstanceFunc();
}

AccelerometerMeasure::AccelerometerMeasure() {
	_accel_history.resize(HISTORY_SIZE);
	_accelerometer_measurements.resize(6000);

	_sensor_manager = AcquireASensorManagerInstance();
	_accelerometer = ASensorManager_getDefaultSensor(_sensor_manager, ASENSOR_TYPE_ACCELEROMETER);
	_accelerometer_looper = ALooper_prepare(0);
	assert(_accelerometer_looper != nullptr);
	_accelerometer_event_queue = ASensorManager_createEventQueue(_sensor_manager,
	                                                             _accelerometer_looper,
	                                                             3, &accelerometer_callback, this);
	assert(_accelerometer_event_queue != nullptr);
	auto status = ASensorEventQueue_enableSensor(_accelerometer_event_queue,
	                                             _accelerometer);
	assert(status >= 0);
	status = ASensorEventQueue_setEventRate(_accelerometer_event_queue,
	                                        _accelerometer,
	                                        ASensor_getMinDelay(_accelerometer));

}

void AccelerometerMeasure::startMeasure() {
	_base_accel = baseAccelerometerValue();
	_startMeasuring = true;
}

void AccelerometerMeasure::stopMeasure() {
	_startMeasuring = false;

	cv::KalmanFilter kalmanFilter{9, 3, 0, CV_64F};
	//F
	kalmanFilter.transitionMatrix = cv::Mat(9,9,CV_64F, cv::Scalar::all(0));
	for(int j = 0; j < 9;++j){
		kalmanFilter.transitionMatrix.at<double>(j,j)= 1.0;
	}
	for(int j = 0; j < 3; ++j) {
		kalmanFilter.transitionMatrix.at<double>(j, j+3) = _kalman_delta_t;
		kalmanFilter.transitionMatrix.at<double>(j, j+6) = _kalman_delta_t*_kalman_delta_t*0.5;
	}
	kalmanFilter.statePost = cv::Mat::zeros(9, 1,CV_64F);

	kalmanFilter.controlMatrix = cv::Mat(9,3,CV_64F, cv::Scalar::all(0));
	kalmanFilter.measurementMatrix = cv::Mat::zeros(3, 9, CV_64F);
	for(int j = 0; j< 3; ++j)
	{
		kalmanFilter.measurementMatrix.at<double>(j,6+j)=1;
	}
	cv::Mat estimated{};

	cv::Vec3d pos{0,0,0};
	cv::Vec3d vel{0,0,0};
	for (size_t i = 1; i < _accelerometer_measurements.size(); ++i) {
		cv::Mat mesMat(3,1,CV_64F);
		auto accel = _accelerometer_measurements[i].value-_base_accel;
		mesMat.at<double>(0) = accel(0);
		mesMat.at<double>(1) = accel(1);
		mesMat.at<double>(2) = accel(2);
		estimated = kalmanFilter.correct(mesMat);

		double deltaT = (double)std::chrono::duration_cast<std::chrono::nanoseconds>
		    (_accelerometer_measurements[i].timestamp-_accelerometer_measurements[i-1].timestamp).count();
		deltaT /= std::nano::den;
		vel += accel*deltaT;
		pos += accel*0.5*deltaT*deltaT+vel*deltaT;
	};

	LOGI("%lf, %lf, %lf",pos(0),pos(1),pos(2));
	_accelerometer_measurements.clear();
}

AccelerometerMeasure::~AccelerometerMeasure() {
	ALooper_release(_accelerometer_looper);
	ASensorManager_destroyEventQueue(_sensor_manager, _accelerometer_event_queue);
}

