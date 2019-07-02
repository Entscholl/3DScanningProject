
#include "StdHeader.h"

#include <android/native_window_jni.h>

#include <opencv2/core.hpp>
#include <omp.h>
#include <opencv2/imgproc/types_c.h>
#include "AccelerometerMeasuring.h"
#include "CameraStuff.h"
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
JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_processImages(JNIEnv *env, jobject,
        jlong addrInputA, jlong addrInputB, jlong addrOutputMat) {
    LOGI("Starting Image Processing");
    double start = omp_get_wtime();
    //cv::Mat test(StereoReconstruction::Camera::current_picture_width ,
    //             StereoReconstruction::Camera::current_picture_height,CV_8UC3,
    //             StereoReconstruction::Camera::current_picture.get(),
    //             StereoReconstruction::Camera::current_picture_width * 1);


    cv::Mat *output = reinterpret_cast<cv::Mat*>(addrOutputMat);
    cv::Mat *inputA = reinterpret_cast<cv::Mat*>(addrInputA);
    cv::Mat *inputB = reinterpret_cast<cv::Mat*>(addrInputB);
	/*
    cv::Mat A;
    cv::Mat B;
    cv::Mat temp_result;
    cv::cvtColor(*inputA, A, CV_BGR2GRAY);
    cv::cvtColor(*inputB, B, CV_BGR2GRAY);
    cv::Mat cameraMatrix[2], distCoeffs[2];
    */
    /*
    cameraMatrix[0] = cv::initCameraMatrix2D({},{},inputA->size(),0);
    cameraMatrix[1] = cv::initCameraMatrix2D({},{},inputA->size(),0);
    cv::Mat R, T, E, F;

    double rms = stereoCalibrate({}, {}, {},
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 inputA->size(), R, T, E, F,
                                 cv::CALIB_FIX_ASPECT_RATIO +
                                 cv::CALIB_ZERO_TANGENT_DIST +
                                 cv::CALIB_USE_INTRINSIC_GUESS +
                                 cv::CALIB_SAME_FOCAL_LENGTH +
                                 cv::CALIB_RATIONAL_MODEL +
                                 cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
                                 cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5) );
    LOGI("Done with stereo calibration RMS: %f", rms);
    */
    /*
    cv::Mat R1, R2, P1, P2, Q, R, T;
    cameraMatrix[0] = cv::Mat::eye(3,3, CV_64F);
    cameraMatrix[1] = cv::Mat::eye(3,3, CV_64F);
    R = cv::Mat::eye(3, 3, CV_64F);
    T = cv::Mat::zeros(3, 1, CV_64F);

    cv::stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  A.size(), R, T, R1, R2, P1, P2, Q,
                  cv::CALIB_ZERO_DISPARITY, 1);

    cv::Mat remap[2][2];

    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, inputA->size(), CV_16SC2,
            remap[0][0], remap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, inputA->size(), CV_16SC2,
            remap[1][0], remap[1][1]);

    cv::Mat A_rectified;
    cv::Mat B_rectified;

    cv::remap(A, A_rectified, remap[0][0], remap[0][1], cv::INTER_LINEAR);
    cv::remap(B, B_rectified, remap[1][0], remap[1][1], cv::INTER_LINEAR);
    //call Rectify(Image A, delta Pose?);
    //call Rectify(Image B, delta Pose);

    //call StereoMatching(Image rekt_A, Image rekt_B)
    auto stereo = cv::StereoBM::create(32,7);
    stereo->compute(A_rectified, B_rectified, temp_result );

    cv::resize(temp_result, *output, cv::Size(1100,2000));
    //call Triangulation(???)
    */
    //return depth_map
    //*output = disparity;
    //Then launch new Activity which displays the result.
    //TODO resize accordingly to display size
	cv::resize(*inputA, *inputA, cv::Size(1100,2000));
    double end = omp_get_wtime();
    LOGI("Image Processing took: %fs", end-start);
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
Java_com_example_stereoreconstruction_MainActivity_startCameraPreview(JNIEnv *env, jobject instance,
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
