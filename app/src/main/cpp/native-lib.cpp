

#include <jni.h>
#include <string>

#include <opencv2/core.hpp>
#include <android/log.h>
#include <omp.h>

#define  LOG_TAG    "libgl2jni"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

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
