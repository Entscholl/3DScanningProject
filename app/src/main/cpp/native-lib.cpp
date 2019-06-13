#include <jni.h>
#include <string>

#include <opencv2/core.hpp>

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_stereoreconstruction_MainActivity_stringFromJNI(
		JNIEnv *env,
		jobject /* this */) {
	cv::Mat testMat(10,10,CV_8UC1);
	std::string hello = "Hello from C++";
	return env->NewStringUTF(hello.c_str());
}
