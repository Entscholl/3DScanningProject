#include "CameraStuff.h"
cv::Mat* StereoReconstruction::Camera::output_mat;
bool StereoReconstruction::Camera::intrinsics_set;
bool StereoReconstruction::Camera::distortion_set;
float StereoReconstruction::Camera::intrinsics[5];
float StereoReconstruction::Camera::distortion[5];

static void on_disconnect(void *context, ACameraDevice *device) {
    LOGI("Camera: %s, is disconnected", ACameraDevice_getId(device));
}
static void on_error(void *context, ACameraDevice *device, int error) {
    LOGE("Error %d on device %s", error, ACameraDevice_getId(device));
}
static void on_ready(void *context, ACameraCaptureSession* session) {
    LOGI("Session ready. %p", session);
}
static void on_active(void *context, ACameraCaptureSession* session) {
    LOGI("Session active. %p", session);
}
static void on_closed(void *context, ACameraCaptureSession* session) {
    LOGI("Session closed. %p", session);
}

static void on_image_taken(void* context, AImageReader* image_reader)
{
    AImage *image = nullptr;
    media_status_t status = AImageReader_acquireNextImage(image_reader, &image);
    if (status != AMEDIA_OK) {
        LOGE("Error acquiring Image ");
    }
    LOGI("Processing picture...");
    std::thread fun([=](){
        int32_t num_planes;
        int32_t width;
        int32_t height;
        int32_t Y_pixel_stride;
        int32_t Y_row_stride;
        int32_t U_pixel_stride;
        int32_t U_row_stride;
        int32_t V_pixel_stride;
        int32_t V_row_stride;
        uint8_t *Y_data = nullptr;
        uint8_t *U_data = nullptr;
        uint8_t *V_data = nullptr;
        int32_t Y_len = 0;
        int32_t U_len = 0;
        int32_t V_len = 0;
        AImage_getNumberOfPlanes(image, &num_planes);
        AImage_getWidth(image, &width);
        AImage_getHeight(image, &height);

        AImage_getPlanePixelStride(image, 0, &Y_pixel_stride);
        AImage_getPlaneRowStride(image, 0, &Y_row_stride);
        AImage_getPlanePixelStride(image, 1, &U_pixel_stride);
        AImage_getPlaneRowStride(image, 1, &U_row_stride);
        AImage_getPlanePixelStride(image, 2, &V_pixel_stride);
        AImage_getPlaneRowStride(image, 2, &V_row_stride);

        AImage_getPlaneData(image, 0, &Y_data, &Y_len);
        AImage_getPlaneData(image, 1, &U_data, &U_len);
        AImage_getPlaneData(image, 2, &V_data, &V_len);
        //assert format is actually YUV420 NV21
        //meaning interleaved U and V
        assert(V_pixel_stride == 2);
        assert(U_pixel_stride == 2);
        assert(Y_pixel_stride == 1);
        assert((U_data - V_data) == 1);
        assert(Y_row_stride == width);
        assert(U_row_stride == width);
        assert(V_row_stride == width);
        assert(((height *3) / 2 * width) == Y_len + V_len+1);

        auto current_data = std::make_unique<std::uint8_t []>(
                static_cast<size_t>(Y_len + V_len + 1));


        std::memcpy(&(current_data.get()[0]), Y_data, static_cast<size_t>(Y_len));
        std::memcpy(&(current_data.get()[Y_len]), V_data, static_cast<size_t>(V_len + 1));
        cv::Mat mYUV((height *3) /2, width, CV_8UC1, (void*) current_data.get());
        //StereoReconstruction::Camera::output_mat->resize(height, width);
        //cv::resize(*StereoReconstruction::Camera::output_mat,
        //        *StereoReconstruction::Camera::output_mat, cv::Size(width,height));
        //assert(StereoReconstruction::Camera::output_mat->cols == width);
        //assert(StereoReconstruction::Camera::output_mat->rows == height);
        //assert(StereoReconstruction::Camera::output_mat->type() == CV_8UC3);
        cv::cvtColor(mYUV, *StereoReconstruction::Camera::output_mat, CV_YUV2RGB_NV21, 3);
        cv::transpose(*StereoReconstruction::Camera::output_mat,
                *StereoReconstruction::Camera::output_mat);
        cv::flip(*StereoReconstruction::Camera::output_mat,
                *StereoReconstruction::Camera::output_mat, 1);


        /*
        LOGI("Y:%p, U:%p, V:%p", Y_data, U_data, V_data);
        for(int y = 0; y < StereoReconstruction::Camera::current_picture_height; y++) {
            for (int x = 0; x < StereoReconstruction::Camera::current_picture_width; x++) {
                uint8_t RGB[3];
                int Y = Y_data[y*Y_row_stride+x*Y_pixel_stride];
                int U = U_data[(y*U_row_stride)/4+x*U_pixel_stride/2] - 128;
                int V = V_data[(y*V_row_stride)/4+x*V_pixel_stride/2] - 128;
                int R = static_cast<int>(Y + 1.370705 * V);
                int G = static_cast<int>(Y - 0.698001 * U  - 0.337633  * V);
                int B = static_cast<int>(Y + 1.732446 * U);
                RGB[0] = (uint8_t) std::clamp(R, 0, 255);
                RGB[1] = (uint8_t) std::clamp(G, 0, 255);
                RGB[2] = (uint8_t) std::clamp(B, 0, 255);

                std::memcpy(&(StereoReconstruction::Camera::current_picture.get()
                [x+ y *StereoReconstruction::Camera::current_picture_width]), RGB, 3);
            }
        }
        */
        LOGI("Processing picture done");
        AImage_delete(image);
    });
    fun.detach();
}
bool is_same_ratio(int32_t width, int32_t height, int32_t  other_width , int32_t other_height) {
    return (width * other_height == other_width  * height);
}
bool StereoReconstruction::Camera::open(uint32_t camera_id) {

    ACameraIdList *camera_id_list = nullptr;

    ACameraManager *camera_manager = ACameraManager_create();
    camera_status_t camera_status = ACameraManager_getCameraIdList(camera_manager, &camera_id_list);
    if(camera_status != ACAMERA_OK) {
        LOGE("Error occured while getting camera id list: %d", camera_status);
        return false;
    }
    if(camera_id_list->numCameras < 1) {
        LOGE("No capture device detected");
        return false;
    }
    LOGI("%d cameras detected", camera_id_list->numCameras);

    std::string selected_camera_id = camera_id_list->cameraIds[camera_id];

    LOGI("Starting camera initialization of camera: %s", selected_camera_id.c_str());

    ACameraMetadata *camera_characteristics;

    camera_status = ACameraManager_getCameraCharacteristics(camera_manager,
            selected_camera_id.c_str(), &camera_characteristics);
    if(camera_status != ACAMERA_OK) {
        LOGE("Error occured while getting camera characteristics of %s: %d",
                selected_camera_id.c_str(), camera_status);
        return false;
    }
    handle_meta_data(camera_characteristics);

    device_callbacks.onDisconnected = on_disconnect;
    device_callbacks.onError = on_error;
    LOGI("Trying to open camera: %s", selected_camera_id.c_str());

    camera_status = ACameraManager_openCamera(camera_manager, selected_camera_id.c_str(),
            &device_callbacks, &camera_device);

    if(camera_status != ACAMERA_OK) {
        LOGE("Error occured while opening camera %s", selected_camera_id.c_str());
        return false;
    }
    LOGI("Camera: %s successfully opened", selected_camera_id.c_str());

    ACameraMetadata_free(camera_characteristics);
    ACameraManager_deleteCameraIdList(camera_id_list);
    ACameraManager_delete(camera_manager);

    return true;
}

void StereoReconstruction::Camera::close() {
    if(output_container != NULL) {
        ACaptureSessionOutputContainer_free(output_container);
        output_container = NULL;
    }
    if(camera_session != NULL) {
        ACameraCaptureSession_close(camera_session);
        camera_session = NULL;
    }
    if(preview.capture_request != NULL) {
        ACaptureRequest_free(preview.capture_request);
        preview.capture_request = NULL;
    }
    if(preview.camera_output_target != NULL) {
        ACameraOutputTarget_free(preview.camera_output_target);
        preview.camera_output_target = NULL;
    }
    if(preview.session_output != NULL) {
        ACaptureSessionOutput_free(preview.session_output);
        preview.session_output = NULL;
    }
    if(capture.capture_request != NULL) {
        ACaptureRequest_free(capture.capture_request);
        capture.capture_request = NULL;
    }
    if(capture.camera_output_target != NULL) {
        ACameraOutputTarget_free(capture.camera_output_target);
        capture.camera_output_target = NULL;
    }
    if(capture.session_output != NULL) {
        ACaptureSessionOutput_free(capture.session_output);
        capture.session_output = NULL;
    }
    if(camera_device != NULL) {
        ACameraDevice_close(camera_device);
        camera_device = NULL;
    }
    if(image_reader != nullptr) {
        AImageReader_delete(image_reader);
        image_reader = nullptr;
    }

}

void StereoReconstruction::Camera::take_picture() {
    camera_status_t camera_status;
    if(camera_session  && capture.capture_request) {
        camera_status = ACameraCaptureSession_capture(camera_session, NULL, 1,
                                                      &capture.capture_request, NULL);
        if (camera_status != ACAMERA_OK) {
            LOGE("Error %d occured while setting repeating captures with camera", camera_status);
        }
    } else {
        LOGE("No valid Session/Capture Request");
    }
}

void StereoReconstruction::Camera::start_preview() {
    camera_status_t camera_status;
    if(camera_session && preview.capture_request) {
        camera_status = ACameraCaptureSession_setRepeatingRequest(camera_session, NULL, 1,
                                                                  &preview.capture_request, NULL);
        if(camera_status != ACAMERA_OK) {
            LOGE("Error %d occured while setting repeating captures with camera", camera_status);
        }
    } else {
        LOGE("No valid Session/Capture Request");
    }
}

void StereoReconstruction::Camera::handle_meta_data(ACameraMetadata *camera_characteristics) {
    camera_status_t camera_status;
    ACameraMetadata_const_entry entry;
    camera_status = ACameraMetadata_getConstEntry(camera_characteristics,
                                  ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &entry);

    if(camera_status == ACAMERA_OK) {
        int32_t current_width = ANativeWindow_getWidth(window);
        int32_t current_height = ANativeWindow_getHeight(window);
        int32_t resize[2] = {1920, 1080};
        supported_formats.reserve(entry.count / 4);
        for (int i = 0; i < entry.count; i += 4) {
            int32_t input = entry.data.i32[i + 3];
            //if (input)
            //    continue;
            int32_t format = entry.data.i32[i + 0];
            int32_t width = entry.data.i32[i + 1];
            int32_t height = entry.data.i32[i + 2];

            if (is_same_ratio(current_width, current_height, width, height) &&
                current_width > width &&
                current_height > height) {
                resize[0] = width;
                resize[1] = height;
            }
            supported_formats.emplace_back(width, height, format);
            std::string format_string = Format::type_to_string(format);
            LOGI("Available Stream Config Input: %d, Size(%d, %d), Format: %s", input, width,
                 height,
                 format_string.c_str());
        }
        LOGI("Chose: Size(%d, %d)", resize[0], resize[1]);
        //Does not work for some reason
        //int result = ANativeWindow_setBuffersGeometry(window, resize[0],resize[1], 0);
        //LOGI("setBuffersGeometry(%d)", result);
    } else {
        LOGI("Could not get scaler stream configurations");
    }

    ACameraMetadata_const_entry intrinics_entry;
    camera_status = ACameraMetadata_getConstEntry(camera_characteristics,
                                                  ACAMERA_LENS_INTRINSIC_CALIBRATION, &intrinics_entry);

    if(camera_status == ACAMERA_OK) {
        intrinsics_set = true;
        std::memcpy((void*) intrinsics, (void*) intrinics_entry.data.f, 5*sizeof(float));
        LOGI("Camera Intrinsics: %f ,%f , %f, %f, %f", intrinics_entry.data.f[0],
             intrinics_entry.data.f[1],intrinics_entry.data.f[2],intrinics_entry.data.f[3],
             intrinics_entry.data.f[4]);
    } else {
        intrinsics_set = false;
        if(camera_status == ACAMERA_ERROR_METADATA_NOT_FOUND) {
            LOGI("Could not get camera intrinsics");
        }
    }

    ACameraMetadata_const_entry lens_distortion_entry;
    camera_status = ACameraMetadata_getConstEntry(camera_characteristics,
                                                  ACAMERA_LENS_DISTORTION, &lens_distortion_entry);
    if(camera_status == ACAMERA_OK) {
        distortion_set = true;
        std::memcpy((void*) distortion, (void*) lens_distortion_entry.data.f, 5*sizeof(float));
        LOGI("Lens Distortion parameters: %f ,%f , %f, %f, %f", lens_distortion_entry.data.f[0],
             lens_distortion_entry.data.f[1],lens_distortion_entry.data.f[2],
             lens_distortion_entry.data.f[3],lens_distortion_entry.data.f[4]);
    } else {
        distortion_set = false;
        if(camera_status == ACAMERA_ERROR_METADATA_NOT_FOUND) {
            LOGI("Could not get camera lens distortion");
        }
    }
}

void StereoReconstruction::Camera::set_up_session(Format capture_format,
                                                  ACameraDevice_request_template preview_template,
                                                  ACameraDevice_request_template capture_template) {
    set_up_capture_request(capture_format, capture_template);
    set_up_preview_request(preview_template);
    camera_status_t camera_status;

    session_callbacks.onReady = on_ready;
    session_callbacks.onActive = on_active;
    session_callbacks.onClosed = on_closed;

    if(output_container) {
        ACaptureSessionOutputContainer_free(output_container);
        output_container= NULL;
    }
    if(camera_session) {
        ACameraCaptureSession_close(camera_session);
        camera_session= NULL;
    }
    if(preview.session_output) {
        ACaptureSessionOutput_free(preview.session_output);
        preview.session_output = NULL;
    }
    if(capture.session_output) {
        ACaptureSessionOutput_free(preview.session_output);
        capture.session_output = NULL;
    }


    ACaptureSessionOutputContainer_create(&output_container);

    if(preview.window) {
        camera_status = ACaptureSessionOutput_create(preview.window, &preview.session_output);
        if (camera_status != ACAMERA_OK) {
            LOGE("Error %d occured while creating session output", camera_status);
        }
        camera_status = ACaptureSessionOutputContainer_add(output_container, preview.session_output);
        if (camera_status != ACAMERA_OK) {
            LOGE("Error %d occured while adding session output", camera_status);
        }
    } else {
        LOGE("No preview window");
    }
    if(capture.window) {
        camera_status = ACaptureSessionOutput_create(capture.window, &capture.session_output);
        if (camera_status != ACAMERA_OK) {
            LOGE("Error %d occured while creating session output", camera_status);
        }
        camera_status = ACaptureSessionOutputContainer_add(output_container, capture.session_output);
        if (camera_status != ACAMERA_OK) {
            LOGE("Error %d occured while adding session output", camera_status);
        }
    } else {
        LOGE("No capture window");
    }


    LOGI("Trying to create capture session");

    camera_status = ACameraDevice_createCaptureSession(camera_device,
                                                       output_container,
                                                       &session_callbacks,
                                                       &camera_session);

    if (camera_status != ACAMERA_OK) {
        LOGE("Error %d occured while creating capture session", camera_status);
        switch (camera_status) {
            case ACAMERA_ERROR_INVALID_PARAMETER:
                LOGE("Any of device, outputs, callbacks or session is NULL");
                break;
            case ACAMERA_ERROR_CAMERA_DISCONNECTED:
                LOGE("The camera device is closed");
                break;
            case ACAMERA_ERROR_CAMERA_DEVICE:
                LOGE("The camera device encountered fatal error.");
                break;
            case ACAMERA_ERROR_CAMERA_SERVICE:
                LOGE("The camera device encountered fatal error.");
                break;
            case ACAMERA_ERROR_UNKNOWN:
                LOGE("Failed for some other reason");
                break;
            default:
                LOGE("Unknown Error Code");
                break;
        }
        return;
    }

}

void StereoReconstruction::Camera::set_up_capture_request(Format capture_format,
        ACameraDevice_request_template capture_template) {
    camera_status_t camera_status;
    media_status_t media_status;
    media_status = AImageReader_new(capture_format.width, capture_format.height,
                                    capture_format.type, 1, &image_reader);

    if (media_status != AMEDIA_OK) {
        LOGE("Error creating Image Reader");
    }

    AImageReader_ImageListener listener{
            .context = nullptr,
            .onImageAvailable = on_image_taken,
    };

    AImageReader_setImageListener(image_reader, &listener);

    AImageReader_getWindow(image_reader, &capture.window);

    LOGI("Trying to create capture capture request");

    camera_status = ACameraDevice_createCaptureRequest(camera_device, capture_template,
                                                       &capture.capture_request);

    if(camera_status != ACAMERA_OK) {
        LOGE("Error occured while creating capture capture request");
        return;
    }
    LOGI("Trying to create/add capture capture target");

    camera_status = ACameraOutputTarget_create(capture.window, &capture.camera_output_target);
    if(camera_status != ACAMERA_OK) {
        LOGE("Error occured while creating capture output target");
        return;
    }
    camera_status = ACaptureRequest_addTarget(capture.capture_request, capture.camera_output_target);
    if(camera_status != ACAMERA_OK) {
        LOGE("Error occured while adding capture target");
        return;
    }
}

void StereoReconstruction::Camera::set_up_preview_request(ACameraDevice_request_template
preview_template) {
    camera_status_t camera_status;
    int32_t width = ANativeWindow_getWidth(window);
    int32_t height = ANativeWindow_getHeight(window);
    if(std::find_if(supported_formats.begin(), supported_formats.end(), [&] (const Format& format) {
        //return is_same_ratio(width, height, format.width, format.height);
        return width == format.width && height == format.height;
    }) == supported_formats.end()) {
        LOGE("Unsupported Format");
        return;
    }
    preview.window = window;
    LOGI("Trying to create preview capture request");

    camera_status = ACameraDevice_createCaptureRequest(camera_device,
                                                       preview_template, &preview.capture_request);

    if(camera_status != ACAMERA_OK) {
        LOGE("Error occured while creating preview capture request");
        return;
    }
    LOGI("Trying to create/add preview capture target");
    camera_status = ACameraOutputTarget_create(preview.window, &preview.camera_output_target);
    if(camera_status != ACAMERA_OK) {
        LOGE("Error occured while creating preview output target");
        return;
    }
    camera_status = ACaptureRequest_addTarget(preview.capture_request, preview.camera_output_target);
    if(camera_status != ACAMERA_OK) {
        LOGE("Error occured while adding capture target");
        return;
    }
}
