
#include "StdHeader.h"

#include <android/native_window_jni.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include <omp.h>
#include "AccelerometerMeasuring.h"
#include "CameraStuff.h"
#include "StereoDepthPipeline.h"
extern "C" {

JNIEXPORT jstring JNICALL
Java_com_example_stereoreconstruction_MainActivity_stringFromJNI(
		JNIEnv *env,
		jobject /* this */) {
#pragma omp parallel for
	for (int i = 0; i < 1 << 16; i++) {
		LOGI("Hello from Thread #%d", omp_get_thread_num());
	}
	cv::Mat testMat(10, 10, CV_8UC1);
	std::string hello = "Hello from C++";
	return env->NewStringUTF(hello.c_str());
}
JNIEXPORT jint JNICALL
Java_com_example_stereoreconstruction_MainActivity_processImages(JNIEnv *env, jobject,
        jlong addrInputA, jlong addrInputB, jlong addrOutputMat) {
    LOGI("Starting Image Processing");
    double start = omp_get_wtime();


    cv::Mat *output = reinterpret_cast<cv::Mat*>(addrOutputMat);
    cv::Mat *inputA = reinterpret_cast<cv::Mat*>(addrInputA);
    cv::Mat *inputB = reinterpret_cast<cv::Mat*>(addrInputB);
    if(inputA->rows == 0 || inputA->cols == 0 ||
        inputB->rows == 0 || inputB->cols == 0) {
        return -1;
    }
    StereoReconstruction::StereoDepthPipeline& pipeline = StereoReconstruction::StereoDepthPipeline::instance();
    pipeline.set_input_A(inputA);
    pipeline.set_input_B(inputB);
    pipeline.stereo_match(output);

    double end = omp_get_wtime();
    LOGI("Image Processing took: %fs", end-start);
    return 0;
}
JNIEXPORT jint JNICALL
Java_com_example_stereoreconstruction_MainActivity_rectifyImages(JNIEnv *env, jobject,
                                                                 jlong addrInputA, jlong addrInputB) {
    LOGI("Starting Image Processing");
    double start = omp_get_wtime();

    cv::Mat *inputA = reinterpret_cast<cv::Mat*>(addrInputA);
    cv::Mat *inputB = reinterpret_cast<cv::Mat*>(addrInputB);
    if(inputA->rows == 0 || inputA->cols == 0 ||
       inputB->rows == 0 || inputB->cols == 0) {
        return -1;
    }
    StereoReconstruction::StereoDepthPipeline& pipeline = StereoReconstruction::StereoDepthPipeline::instance();
    pipeline.set_input_A(inputA);
    pipeline.set_input_B(inputB);
    cv::Mat cameraMatrix[2], distCoeffs[2];
    for(int i = 0; i <2; i++) {
        distCoeffs[i].ones(1, 5, CV_64F);
        if(StereoReconstruction::Camera::distortion_set) {
            distCoeffs[i].zeros(1, 5, CV_64F);
            // kappa_1
            distCoeffs[i].at<double>(0, 0) = StereoReconstruction::Camera::distortion[0];
            // kappa_2
            distCoeffs[i].at<double>(0, 1) = StereoReconstruction::Camera::distortion[1];
            // tangential lens distortion
            distCoeffs[i].at<double>(0, 2) = StereoReconstruction::Camera::distortion[3];
            // tangential lens distortion
            distCoeffs[i].at<double>(0, 3) = StereoReconstruction::Camera::distortion[4];
            // kappa_3
            distCoeffs[i].at<double>(0, 4) = StereoReconstruction::Camera::distortion[2];
        } else {

        }
        cameraMatrix[i] = cv::Mat::eye(3,3, CV_64F);
        if(StereoReconstruction::Camera::intrinsics_set) {
            // f_x
            cameraMatrix[i].at<double>(0, 0) = StereoReconstruction::Camera::intrinsics[0];
            // f_y
            cameraMatrix[i].at<double>(1, 1) = StereoReconstruction::Camera::intrinsics[1];
            // c_x
            cameraMatrix[i].at<double>(0, 2) = StereoReconstruction::Camera::intrinsics[2];
            // c_y
            cameraMatrix[i].at<double>(1, 2) = StereoReconstruction::Camera::intrinsics[3];
            // s
            cameraMatrix[i].at<double>(0, 1) = StereoReconstruction::Camera::intrinsics[4];
        } else {
            // f_x
            cameraMatrix[i].at<double>(0, 0) = 502.59;
            // f_y
            cameraMatrix[i].at<double>(1, 1) = 502.63;
            // c_x
            cameraMatrix[i].at<double>(0, 2) = 315.87;
            // c_y
            cameraMatrix[i].at<double>(1, 2) = 234.41;
            // s
            cameraMatrix[i].at<double>(0, 1) = 0;
            //radial [9,992e-02, -2,22e-01]
    }
    pipeline.set_camera_matrix_A(cameraMatrix[0]);
    pipeline.set_camera_matrix_B(cameraMatrix[1]);
    pipeline.set_distortion_coefficients_A(distCoeffs[0]);
    pipeline.set_distortion_coefficients_B(distCoeffs[1]);
    pipeline.set_translate_vector(AccelerometerMeasure::translation);

    pipeline.rectify();

    double end = omp_get_wtime();
    LOGI("Image Rectifying took: %fs", end-start);
    return 0;
}


JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_initMeasurement(JNIEnv *env, jobject) {
	LOGI("Init Measurement");
	const auto x = AccelerometerMeasure::getInstance();
}
JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_startMeasurement(JNIEnv *env, jobject) {
	LOGI("Start Measurement");
	AccelerometerMeasure::getInstance()->startMeasure();
}
JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_stopMeasurement(JNIEnv *env, jobject) {
	LOGI("Stop Measurement");
	AccelerometerMeasure::getInstance()->stopMeasure();
}
JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_startCameraPreview(JNIEnv *env, jobject instance) {
    StereoReconstruction::Camera::start_preview_();
}
JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_setUpCameraSession(JNIEnv *env, jobject instance,
                                                                      jobject surfaceView) {
    StereoReconstruction::Camera::set_up_session(env, surfaceView);
}

JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_initCamera(JNIEnv *env, jobject instance,
                                                              jobject surfaceView) {
    StereoReconstruction::Camera::init(env, surfaceView);
}
JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_stopCameraPreview(JNIEnv *env,
                                                                     jobject instance) {
    StereoReconstruction::Camera::stop();
}
JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_takePicture(JNIEnv *env, jobject instance, jlong
    output_mat_addr) {
    cv::Mat *output = reinterpret_cast<cv::Mat*>(output_mat_addr);
	StereoReconstruction::Camera::take_picture(output);
}
}
