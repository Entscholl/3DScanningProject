
#include "AccelerometerMeasuring.h"

#include <stdlib.h>
#include <dlfcn.h>

#include <opencv2/video/tracking.hpp>

constexpr double baselineAlpha=0.2;

int accelerometer_callback(int fd, int events, void *data) {
	ASensorEvent event;
	auto *_this = (AccelerometerMeasure *) data;
	while (ASensorEventQueue_getEvents(_this->_accelerometer_event_queue, &event, 1)) {
		if (event.type == ASENSOR_TYPE_ACCELEROMETER) {
			cv::Vec3d accVec{event.acceleration.x, event.acceleration.y, event.acceleration.z};
			if (accVec.dot(accVec) < 0.1) {
				return 1;
			}
			if (_this->_startMeasuring && !_this->_is_first_orientation) {
				_this->_accelerometer_measurements.push_back({std::move(accVec),
				                                              _this->_last_Orientation,
				                                              std::chrono::high_resolution_clock::now()});
			} else {
				_this->_accel_baseline = baselineAlpha*accVec+(1-baselineAlpha)*_this->_accel_baseline;
				//LOGI("Accel, baseline %f, %f, %f", _this->_accel_baseline(0),_this->_accel_baseline(1),_this->_accel_baseline(2));
			}
		}
		if (event.type == ASENSOR_TYPE_ROTATION_VECTOR) {
			Quaternion<float> orientation{event.data[3],event.data[0], event.data[1], event.data[2]};
			_this->_last_Orientation = orientation;
			//LOGI("Quat, baseline %f, %f, %f, %f", event.data[3],event.data[0], event.data[1], event.data[2]);
			if (_this->_is_first_orientation) {
				_this->_is_first_orientation = false;
				_this->_start_Orientation =  orientation;
			} else {
				_this->_final_Orientation = orientation;
			}
		}
	}
	return 1;
}

cv::Mat getRotationMatrixFromQuaternion(Quaternion<float> &quat) {
	cv::Mat rot(3,3,CV_32F);
	auto mag = quat.magnitude();
	Quaternion n = quat;
	n.x /= mag;
	n.y /= mag;
	n.z /= mag;
	n.w /= mag;
	rot.at<float>(0,0)=1-2*(n.y*n.y+n.z*n.z);
	rot.at<float>(0,1)=2*(n.x*n.y-n.w*n.z);
	rot.at<float>(0,2)=2*(n.x*n.z+n.w*n.y);


	rot.at<float>(1,0)=2*(n.x*n.y+n.w*n.z);
	rot.at<float>(1,1)=1-2*(n.x*n.x+n.z*n.z);
	rot.at<float>(1,2)=2*(n.y*n.z-n.w*n.x);


	rot.at<float>(2,0)=2*(n.x*n.z-n.w*n.x);
	rot.at<float>(2,1)=2*(n.y*n.z+n.w*n.x);
	rot.at<float>(2,2)=1-2*(n.x*n.x+n.y*n.y);
	LOGI("%f",rot.at<float>(1,1));
	return rot;
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
	assert(status == 0);

	_orientation_sensor = ASensorManager_getDefaultSensor(
			_sensor_manager, ASENSOR_TYPE_ROTATION_VECTOR);
	assert(_orientation_sensor != nullptr);
	status = ASensorEventQueue_enableSensor(_accelerometer_event_queue, _orientation_sensor);
	assert(status == 0);

	status = ASensorEventQueue_setEventRate(_accelerometer_event_queue,
	                                        _accelerometer,
	                                        ASensor_getMinDelay(_accelerometer));
	status = ASensorEventQueue_setEventRate(_accelerometer_event_queue,
	                                        _orientation_sensor,
	                                        ASensor_getMinDelay(_orientation_sensor));

}

void AccelerometerMeasure::startMeasure() {
	_base_accel = baseAccelerometerValue();
	_accelerometer_measurements.clear();
	_startMeasuring = true;
	_is_first_orientation = true;

}

cv::Matx44f AccelerometerMeasure::stopMeasure() {
	_startMeasuring = false;

	cv::Vec3d pos{0, 0, 0};
	cv::Vec3d vel{0, 0, 0};

	cv::Vec3d accelFilt {0,0,0};
	for (size_t i = 1; i < _accelerometer_measurements.size(); ++i) {
		cv::Mat mesMat(3, 1, CV_64F);
		if(_accelerometer_measurements[i].value(0) < 0.01 &&
				_accelerometer_measurements[i].value(1) < 0.01 &&
				_accelerometer_measurements[i].value(2) < 0.01
				){
			continue;
		}
		auto deltaOrientation = _accelerometer_measurements[i].orientation*_start_Orientation.inverse();
		//deltaOrientation = Quaternion<float>(-deltaOrientation.w,deltaOrientation.x,deltaOrientation.y,-deltaOrientation.z);
		auto accel = _accelerometer_measurements[i].value - _base_accel;
		double deltaT = (double) std::chrono::duration_cast<std::chrono::nanoseconds>
				(_accelerometer_measurements[i].timestamp -
				 _accelerometer_measurements[i - 1].timestamp).count();
		deltaT /= std::nano::den;
		accelFilt = accel*0.5+0.5*accelFilt;
		if (accelFilt.dot(accelFilt) < 0.02) {
			accelFilt = {0, 0, 0};
		}
		vel += accelFilt * deltaT;
		pos += accelFilt * 0.5 * deltaT * deltaT + vel * deltaT;

	};


	auto rotQuat = _final_Orientation*_start_Orientation.inverse();

	auto rot = getRotationMatrixFromQuaternion(rotQuat);

	LOGI("Pos: %lf, %lf, %lf", pos(0), pos(1), pos(2));
	LOGI("Rot: %f, %f, %f", rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2));
	LOGI("Rot: %f, %f, %f", rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2));
	LOGI("Rot: %f, %f, %f", rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

	cv::Mat transform = cv::Mat::eye(4,4,CV_32F);
	transform(cv::Range(0,3),cv::Range(0,3)) = rot;
	transform.at<float>(0,3)= static_cast<float>(pos(0));
	transform.at<float>(1,3)= static_cast<float>(pos(1));
	transform.at<float>(2,3) = static_cast<float>(pos(2));
	return transform;
}

AccelerometerMeasure::~AccelerometerMeasure() {
	ALooper_release(_accelerometer_looper);
	ASensorManager_destroyEventQueue(_sensor_manager, _accelerometer_event_queue);
}

