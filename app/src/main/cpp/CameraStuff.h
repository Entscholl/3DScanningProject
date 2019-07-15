
#ifndef INC_3DSCANNINGPROJECT_CAMERASTUFF_H
#define INC_3DSCANNINGPROJECT_CAMERASTUFF_H
#include "StdHeader.h"
#include <cstddef>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>
#include <android/native_window_jni.h>
#include <media/NdkImageReader.h>
#include <media/NdkImage.h>
#include <memory>
#include <optional>
#include <vector>
#include <thread>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

namespace StereoReconstruction {
    class Camera {
    private:
        struct Request {
            ANativeWindow *window = NULL;
            ACameraOutputTarget *camera_output_target = NULL;
            ACaptureRequest *capture_request = NULL;
            ACaptureSessionOutput *session_output = NULL;
        };
        struct Format {
            uint32_t  type;
            uint32_t  width;
            uint32_t  height;

            Format (uint32_t width, uint32_t height, uint32_t type):
                    type(type), width(width), height(height){}
            const static std::string type_to_string(uint32_t format) {
                std::string format_string;
                switch(format) {
                    case AIMAGE_FORMAT_JPEG:
                        format_string = "JPEG";
                        break;
                    case AIMAGE_FORMAT_DEPTH16:
                        format_string = "DEPTH16";
                        break;
                    case AIMAGE_FORMAT_DEPTH_JPEG:
                        format_string = "DEPTH_JPEG";
                        break;
                    case AIMAGE_FORMAT_DEPTH_POINT_CLOUD:
                        format_string = "DEPTH_POINT_CLOUD";
                        break;
                    case AIMAGE_FORMAT_HEIC:
                        format_string = "HEIC";
                        break;
                    case AIMAGE_FORMAT_RAW_PRIVATE:
                        format_string ="RAW_PRIVATE";
                        break;
                    case AIMAGE_FORMAT_RAW10:
                        format_string ="RAW10";
                        break;
                    case AIMAGE_FORMAT_RAW12:
                        format_string ="RAW12";
                        break;
                    case AIMAGE_FORMAT_RAW16:
                        format_string ="RAW16";
                        break;
                    case AIMAGE_FORMAT_RGB_565:
                        format_string ="RGB_565";
                        break;
                    case AIMAGE_FORMAT_RGB_888:
                        format_string ="RGB_888";
                        break;
                    case AIMAGE_FORMAT_RGBA_8888:
                        format_string ="RGBA_8888";
                        break;
                    case AIMAGE_FORMAT_RGBA_FP16:
                        format_string ="RGBA_FP16";
                        break;
                    case AIMAGE_FORMAT_RGBX_8888:
                        format_string ="RGBX_8888";
                        break;
                    case AIMAGE_FORMAT_Y8:
                        format_string ="Y8";
                        break;
                    case AIMAGE_FORMAT_YUV_420_888:
                        format_string ="YUV_420_888";
                        break;
                    default:
                        format_string = std::to_string(format);
                }
                return format_string;
            }
        };
    public:
        Camera() = default;
        Camera(Camera&) = delete;
        Camera& operator=(Camera&) = delete;
    private:
        std::mutex mutex;
        ANativeWindow *window = nullptr;
        ACameraDevice *camera_device = nullptr;
        AImageReader* image_reader = nullptr;

        ACaptureSessionOutputContainer *output_container = NULL;
        ACameraCaptureSession *camera_session = NULL;
        ACameraCaptureSession_stateCallbacks session_callbacks;

        ACameraDevice_StateCallbacks device_callbacks;
        Request preview;
        Request capture;
        std::vector<Format> supported_formats;
    private:
        bool open(uint32_t camera_id);
        void set_up_session(Format capture_format,
                ACameraDevice_request_template preview_template = TEMPLATE_PREVIEW,
                ACameraDevice_request_template capture_template = TEMPLATE_STILL_CAPTURE);
        void set_up_preview_request(ACameraDevice_request_template preview_template
        = TEMPLATE_PREVIEW);
        void set_up_capture_request(Format capture_format,
                ACameraDevice_request_template capture_template = TEMPLATE_STILL_CAPTURE);
        void take_picture();
        void start_preview();
        void close();
        void handle_meta_data(ACameraMetadata *camera_characteristics);
    public:
        static cv::Mat *output_mat;
        static bool intrinsics_set;
        static bool distortion_set;
        /*
         * intrinsic calibration parameters: [f_x, f_y, c_x, c_y, s]
         */
        static float intrinsics[5];
        /*
         * lens distortion parameters: [kappa_1, kappa_2, kappa_3] and tangential distortion c
         */
        static float distortion[5];

        static Camera& instance() {
            static std::unique_ptr<Camera> instance(new Camera);
            return *instance;
        }
        static void init(JNIEnv* const env, const jobject surface) {
            std::lock_guard<std::mutex> lock_guard(instance().mutex);
            instance().window = ANativeWindow_fromSurface(env, surface);
            //TODO currently using fixed camera 0
            if(!instance().open(0)) return;
            LOGI("Surface in native started");
        }
        static void set_up_session(JNIEnv* const env, const jobject surface) {
            std::lock_guard<std::mutex> lock_guard(instance().mutex);
            instance().window = ANativeWindow_fromSurface(env, surface);
            //TODO hardcoded image size
            instance().set_up_session(Format(1920, 1080, AIMAGE_FORMAT_YUV_420_888));
            //instance().set_up_session(Format(640, 480, AIMAGE_FORMAT_YUV_420_888));
        }
        static void start_preview_() {
            std::lock_guard<std::mutex> lock_guard(instance().mutex);
            instance().start_preview();
        }
        static void stop() {
            std::lock_guard<std::mutex> lock_guard(instance().mutex);
            instance().close();
            if(instance().window != NULL) {
                ANativeWindow_release(instance().window);
                instance().window = NULL;
            }
            LOGI("Surface in native stopped");
        }
        static void take_picture(cv::Mat *_output) {
            std::lock_guard<std::mutex> lock_guard(instance().mutex);
            output_mat = _output;
            instance().take_picture();
        }
    };
}
#endif //INC_3DSCANNINGPROJECT_CAMERASTUFF_H
