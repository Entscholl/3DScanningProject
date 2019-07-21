//
// Created by Michael on 17.06.2019.
//

#ifndef INC_3DSCANNINGPROJECT_STDHEADER_H
#define INC_3DSCANNINGPROJECT_STDHEADER_H
#include <jni.h>
#include <string>
#include <android/log.h>
#if defined(__ARM_NEON__)
/* GCC-compatible compiler, targeting ARM with NEON */
    #include <arm_neon.h>
#endif
#define  LOG_TAG    "lib3dscanning"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

#define PACKAGE_NAME "com.example.stereoreconstruction"

#endif //INC_3DSCANNINGPROJECT_STDHEADER_H
