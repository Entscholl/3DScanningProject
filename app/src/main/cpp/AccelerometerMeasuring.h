//
// Created by Michael on 17.06.2019.
//

#ifndef INC_3DSCANNINGPROJECT_ACCELEROMETERMEASURING_H
#define INC_3DSCANNINGPROJECT_ACCELEROMETERMEASURING_H

#include "StdHeader.h"
#include <android/sensor.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <atomic>
#include <chrono>
#include "Quaternion.h"

class AccelerometerMeasure {
		friend int accelerometer_callback(int fd, int event, void * data);
		friend int gyroscope_callback(int fd, int event, void * data);

		ASensorManager * _sensor_manager;
		const ASensor * _accelerometer;
		const ASensor * _orientation_sensor;
		ALooper* _accelerometer_looper;
		ASensorEventQueue* _accelerometer_event_queue;

		static constexpr uint16_t HISTORY_SIZE = 50;
		cv::Vec3d _accel_baseline ={0,0,0};
		cv::Vec3d _base_accel;

		std::atomic<bool> _startMeasuring;

		std::atomic<bool> _timestamp_delta_t_valid = false;

		struct MeasurementData{
				cv::Vec3d value;
				Quaternion<float> orientation;
				std::chrono::high_resolution_clock::time_point timestamp;
		};
		std::vector<MeasurementData> _accelerometer_measurements;

		Quaternion<float> _start_Orientation;
		Quaternion<float> _final_Orientation;
		Quaternion<float> _last_Orientation;

		bool _is_first_orientation = true;
public:
		AccelerometerMeasure();

		~AccelerometerMeasure();
		void startMeasure();

		cv::Matx44f stopMeasure();

		cv::Vec3d baseAccelerometerValue() const{
			return _accel_baseline;
		}
public:
		static cv::Matx33d rotation;
		static cv::Vec3d translation;
		static AccelerometerMeasure* getInstance() {
			static auto _sAccelSingleton = std::make_unique<AccelerometerMeasure>();
			return _sAccelSingleton.get();
		}
};
cv::Mat getRotationMatrixFromQuaternion(Quaternion<float> &quat);

#endif //INC_3DSCANNINGPROJECT_ACCELEROMETERMEASURING_H
