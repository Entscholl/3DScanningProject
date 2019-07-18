
#include "StdHeader.h"

#include <android/native_window_jni.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/features2d.hpp>

#include <omp.h>
#include "AccelerometerMeasuring.h"
#include "CameraStuff.h"
#include "StereoDepthPipeline.h"
#include "Calibration.h"

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
        jlong addrInputA, jlong addrInputB, jlong addrOutputMat, jint num_disparities,
        jint block_size) {
    LOGI("Starting Stereo matching with %d disparities and %d blocksize", num_disparities, block_size);
    double start = omp_get_wtime();


    cv::Mat *output = reinterpret_cast<cv::Mat*>(addrOutputMat);
    cv::Mat *inputA = reinterpret_cast<cv::Mat*>(addrInputA);
    cv::Mat *inputB = reinterpret_cast<cv::Mat*>(addrInputB);
    if(inputA->rows == 0 || inputA->cols == 0 ||
        inputB->rows == 0 || inputB->cols == 0) {
        return -1;
    }

    StereoReconstruction::StereoDepthPipeline& pipeline = StereoReconstruction::StereoDepthPipeline::instance();
    pipeline.set_num_disparities(num_disparities);
    pipeline.set_block_size(block_size);
    pipeline.set_input_A(inputA);
    pipeline.set_input_B(inputB);
    pipeline.stereo_match(output);


    double end = omp_get_wtime();
    LOGI("Image Processing took: %fs", end-start);
    return 0;
}
JNIEXPORT jint JNICALL
Java_com_example_stereoreconstruction_MainActivity_rectifyImages(JNIEnv *env, jobject,
                                                                 jlong addrInputA, jlong addrInputB,
                                                                 jlong addrOutputMat, jfloat x,
                                                                 jfloat y, jfloat z, jboolean use_gyro,
                                                                 jboolean use_accel, jboolean use_uncalibrated) {
    LOGI("Starting Image Rectification");
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
    cv::Mat cameraMatrix[2], distortion_coefficents[2];
    for(int i = 0; i <2; i++) {
        distortion_coefficents[i] = cv::Mat::zeros(1, 5, CV_64F);
        if(StereoReconstruction::Camera::distortion_set) {
            distortion_coefficents[i].zeros(1, 5, CV_64F);
            // kappa_1
            distortion_coefficents[i].at<double>(0, 0) = StereoReconstruction::Camera::distortion[0];
            // kappa_2
            distortion_coefficents[i].at<double>(0, 1) = StereoReconstruction::Camera::distortion[1];
            // tangential lens distortion
            distortion_coefficents[i].at<double>(0, 2) = StereoReconstruction::Camera::distortion[3];
            // tangential lens distortion
            distortion_coefficents[i].at<double>(0, 3) = StereoReconstruction::Camera::distortion[4];
            // kappa_3
            distortion_coefficents[i].at<double>(0, 4) = StereoReconstruction::Camera::distortion[2];
        } /*else {
            // kappa_1
            distortion_coefficents[i].at<double>(0, 0) = 0.25;
            // kappa_2
            distortion_coefficents[i].at<double>(0, 1) = -1.5;
            // tangential lens distortion
            distortion_coefficents[i].at<double>(0, 2) = 0.0;
            // tangential lens distortion
            distortion_coefficents[i].at<double>(0, 3) = 0.0;
            // kappa_3
            distortion_coefficents[i].at<double>(0, 4) = 2.5;
        }
        */
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
            // 1920 x 1080
            // f_x
            cameraMatrix[i].at<double>(0, 0) = 1455;
            // f_y
            cameraMatrix[i].at<double>(1, 1) = 1455;
            // c_x
            cameraMatrix[i].at<double>(0, 2) = inputA->cols/2;
            // c_y
            cameraMatrix[i].at<double>(1, 2) = inputA->rows/2;
            // s
            cameraMatrix[i].at<double>(0, 1) = 0;
            //TODO Actually calibrated values (Those are correct for some devices)
        }
    }
    pipeline.set_camera_matrix_A(cameraMatrix[0]);
    pipeline.set_camera_matrix_B(cameraMatrix[1]);
    pipeline.set_distortion_coefficients_A(distortion_coefficents[0]);
    pipeline.set_distortion_coefficients_B(distortion_coefficents[1]);
    if(use_gyro) {
        pipeline.set_rotation_matrix(AccelerometerMeasure::rotation);
    } else {
        pipeline.set_rotation_matrix(cv::Matx33f::eye());
    }
    if(use_accel) {
        pipeline.set_translate_vector(AccelerometerMeasure::translation);
    } else {
        pipeline.set_translate_vector(cv::Vec3f(x, y, z));
    }
    if(use_uncalibrated) {
        pipeline.rectify_translation_estimate();
    } else {
        pipeline.rectify();
    }
    cv::hconcat(*inputA, *inputB, *output);

    double end = omp_get_wtime();
    LOGI("Image Rectifying took: %fs", end-start);
    return 0;
}
std::vector<cv::Mat> calibration_images;
JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_addCalibrationImage(JNIEnv *env, jobject) {
    calibration_images.emplace_back();
    StereoReconstruction::Camera::take_picture(&calibration_images.back());
}

JNIEXPORT void JNICALL
Java_com_example_stereoreconstruction_MainActivity_calibrate(JNIEnv *env, jobject) {
    StereoReconstruction::Calibration calibration;
    if(calibration.calibrate(calibration_images)) {
        cv::Mat camera_matrix = calibration.get_camera_matrix();
        cv::Mat distortion_coefficients = calibration.get_distortion_coefficients();
        LOGI("camera matrix:");
        for(int i = 0; i < 3; i++ ) {
            LOGI("%f\t%f\t%f", camera_matrix.at<double>(i, 0), camera_matrix.at<double>(i, 1),
                 camera_matrix.at<double>(i, 2));
        }
        LOGI("distortion coefficients:");
        for(int x = 0; x < distortion_coefficients.rows; x++) {
            for(int y = 0; y < distortion_coefficients.cols; y++) {
                LOGI("(%d,%d): %f", x, y, distortion_coefficients.at<double>(x, y));
            }
        }
    }
    calibration_images.clear();
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