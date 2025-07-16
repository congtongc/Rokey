#include <jni.h>
#include <string>
#include <android/log.h>

#define LOG_TAG "PenguinLib"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

extern "C" {

JNIEXPORT jboolean JNICALL
Java_com_rokey_parkingapp_utils_PenguinLib_initialize(JNIEnv *env, jobject thiz) {
    LOGI("Penguin library initialized");
    return true;
}

JNIEXPORT jstring JNICALL
Java_com_rokey_parkingapp_utils_PenguinLib_processImage(JNIEnv *env, jobject thiz, jstring image_data) {
    const char* data = env->GetStringUTFChars(image_data, nullptr);
    // 실제로는 여기서 이미지 처리를 수행해야 하지만, 지금은 더미 응답을 반환
    env->ReleaseStringUTFChars(image_data, data);
    return env->NewStringUTF("{\"licensePlate\":\"12가3456\",\"confidence\":95,\"carType\":\"승용차\"}");
}

JNIEXPORT void JNICALL
Java_com_rokey_parkingapp_utils_PenguinLib_cleanup(JNIEnv *env, jobject thiz) {
    LOGI("Penguin library cleaned up");
}

} 