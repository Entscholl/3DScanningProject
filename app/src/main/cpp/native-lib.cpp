
#include "StdHeader.h"


#include <opencv2/core.hpp>
#include <omp.h>
#include "AccelerometerMeasuring.h"


extern "C" JNIEXPORT jstring JNICALL
Java_com_example_stereoreconstruction_MainActivity_stringFromJNI(
		JNIEnv *env,
		jobject /* this */) {
	#pragma omp parallel for
	for(int i = 0; i < 1<<16; i++) {
        LOGI("Hello from Thread #%d", omp_get_thread_num());
    }
	cv::Mat testMat(10,10,CV_8UC1);
	std::string hello = "Hello from C++";
	return env->NewStringUTF(hello.c_str());
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_initMeasurement(JNIEnv * env, jobject) {
	LOGI("Init Measurement");
	const auto x = AccelerometerMeasure::getInstance();
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_startMeasurement(JNIEnv * env, jobject) {
	LOGI("Start Measurement");
	AccelerometerMeasure::getInstance()->startMeasure();
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_stopMeasurement(JNIEnv * env, jobject) {
	LOGI("Stop Measurement");
	AccelerometerMeasure::getInstance()->stopMeasure();
}
