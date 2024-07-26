
#include <jni.h>
#include <android/native_window_jni.h>
#include <android/asset_manager_jni.h>
#include <cstring>
#include <string>
#include <dirent.h>

#include "native_debug.h"
#include "camera_manager.h"
#include "image_process.h"
#include "sensor_manager.h"
#include "GLRenderer.h"

#include <opencv2/opencv.hpp>

#include "svo_system.h"


NDKCamera *cameraObj_ = nullptr;
AImageReader* yuvImageReader_ = nullptr;
ImageProcess* yuvProcess_ = nullptr;
jobject surface_, surface_gl_;
ANativeWindow* mnativewindow_ = nullptr;
sensor_manager* sensorManager_ = nullptr;
GLRenderer* glRender_ = nullptr;

SvoSystem* svo_ = nullptr;

static bool save_pic = false, start_svo = false;
static const char *kDirName = "/sdcard/DCIM/Camera/";
static const char *kFileName = "capture";
static double init_Pose[16] = {1., 0., 0., -0.5,
                                0., 0., -1., 0.5,
                                0., 1., 0., 0.5,
                                0., 0., 0., 1.,};

void drawFrame(AImage* image);

void OnImageCallback(void *ctx, AImageReader *reader) {
//    int32_t format;
//    media_status_t status = AImageReader_getFormat(reader, &format);
//    ASSERT(status == AMEDIA_OK, "Failed to get the media format");
//    LOGI("ImageReader::ImageCallback %0x", format);

    AImage *image = nullptr;
    media_status_t status2 = AImageReader_acquireNextImage(reader, &image);
    ASSERT(status2 == AMEDIA_OK && image, "Image is not available");

    if(start_svo)
    {
        static double last_tm = 0.;
        int64_t timestamp;
        AImage_getTimestamp(image, &timestamp);
        double tm = timestamp * 1e-9;

        if(tm - last_tm > 0.2) {
            last_tm = tm;
            cv::Mat mat = yuvProcess_->GetCVImage(image);
            svo_->add_img(mat, tm);
        }
    }
    else{
        drawFrame(image);
    }

    AImage_delete(image);
}

void drawFrame(AImage* image) {

    ANativeWindow_acquire(mnativewindow_);

    ANativeWindow_Buffer buf;
    if (ANativeWindow_lock(mnativewindow_, &buf, nullptr) < 0) {
        ANativeWindow_release(mnativewindow_);
        return;
    }

    yuvProcess_->DisplayImage(&buf, image);

    if(save_pic) {
        save_pic = false;

        cv::Mat mat = yuvProcess_->GetCVImage(image);

        cv::Mat out_mat;
        cv::cvtColor(mat, out_mat, cv::COLOR_RGBA2BGR);
        //cv::cvtColor(out_mat, out_mat, cv::COLOR_BGRA2BGR);

        DIR *dir = opendir(kDirName);
        if (dir) {
            closedir(dir);
        } else {
            std::string cmd = "mkdir -p ";
            cmd += kDirName;
            system(cmd.c_str());
        }

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        struct tm localTime;
        localtime_r(&ts.tv_sec, &localTime);

        std::string fileName(kDirName);
        std::string dash("-");
        fileName += kFileName + std::to_string(localTime.tm_mon) +
                    std::to_string(localTime.tm_mday) + dash +
                    std::to_string(localTime.tm_hour) +
                    std::to_string(localTime.tm_min) +
                    std::to_string(localTime.tm_sec) + ".jpg";
        cv::imwrite(fileName, out_mat);
    }

    ANativeWindow_unlockAndPost(mnativewindow_);
    ANativeWindow_release(mnativewindow_);

}

void visualize(cv::Mat img, double frame_pos[])
{
    float gl_pos[16];
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++)
            gl_pos[j*4 + i] = frame_pos[i*4 + j];
    }

    glRender_->setPose(gl_pos);

    ANativeWindow_acquire(mnativewindow_);
    ANativeWindow_Buffer buf;
    if (ANativeWindow_lock(mnativewindow_, &buf, nullptr) < 0) {
        ANativeWindow_release(mnativewindow_);
        return;
    }

    yuvProcess_->DisplayImage(&buf, img);

    ANativeWindow_unlockAndPost(mnativewindow_);
    ANativeWindow_release(mnativewindow_);

}

/**
 * createCamera() Create application instance and NDK camera object
 */
extern "C" JNIEXPORT void JNICALL
Java_com_sample_view_ViewActivity_createCamera(JNIEnv *env,
                                               jobject instance,
                                               jint width, jint height) {
  cameraObj_ = new NDKCamera();
  cameraObj_->MatchCaptureSizeRequest(width, height,
                                      nullptr);

  auto view = cameraObj_->GetCompatibleCameraRes();

  const int MAX_BUF_COUNT = 4;
  media_status_t status = AImageReader_new(view.width, view.height, AIMAGE_FORMAT_YUV_420_888, //AIMAGE_FORMAT_JPEG,
                                             MAX_BUF_COUNT, &yuvImageReader_);
  ASSERT(yuvImageReader_ && status == AMEDIA_OK, "Failed to create AImageReader");

  AImageReader_ImageListener listener{
            .context = nullptr,
            .onImageAvailable = OnImageCallback,
  };
  AImageReader_setImageListener(yuvImageReader_, &listener);

  yuvProcess_ = new ImageProcess(90);

//  int32_t facing = 0, angle = 0, imageRotation = 0;
//  if (cameraObj_->GetSensorOrientation(&facing, &angle)) {
//      if (facing == ACAMERA_LENS_FACING_FRONT) {
//          imageRotation = (angle + rotation_) % 360;
//          imageRotation = (360 - imageRotation) % 360;
//      } else {
//          imageRotation = (angle - rotation_ + 360) % 360;
//      }
//  }
//
//  yuvReader_->SetPresentRotation(imageRotation);
}

/**
 * deleteCamera():
 *   releases native application object, which
 *   triggers native camera object be released
 */
extern "C" JNIEXPORT void JNICALL
Java_com_sample_view_ViewActivity_deleteCamera(JNIEnv *env, jobject instance) {
  if (!cameraObj_) {
    return;
  }

  delete cameraObj_;

  // also reset the private global object
  cameraObj_ = nullptr;

  ASSERT(yuvImageReader_, "NULL Pointer to %s", __FUNCTION__);
  AImageReader_delete(yuvImageReader_);
  yuvImageReader_ = nullptr;

}

extern "C" JNIEXPORT jobject JNICALL
Java_com_sample_view_ViewActivity_getMinimumCompatiblePreviewSize(
    JNIEnv *env, jobject instance) {
  if (!cameraObj_) {
    return nullptr;
  }

  jclass cls = env->FindClass("android/util/Size");
  jobject previewSize =
      env->NewObject(cls, env->GetMethodID(cls, "<init>", "(II)V"),
                     cameraObj_->GetCompatibleCameraRes().width,
                     cameraObj_->GetCompatibleCameraRes().height);
  return previewSize;
}

/**
 * getCameraSensorOrientation()
 * @ return camera sensor orientation angle relative to Android device's
 * display orientation. This sample only deal to back facing camera.
 */
extern "C" JNIEXPORT jint JNICALL
Java_com_sample_view_ViewActivity_getCameraSensorOrientation(
    JNIEnv *env, jobject instance) {
  ASSERT(cameraObj_, "NativeObject should not be null Pointer");

   int32_t facing = 0, angle = 0;
   if (cameraObj_->GetSensorOrientation(&facing, &angle) &&
       facing == ACAMERA_LENS_FACING_BACK) {
       return angle;
   }
   ASSERT(false, "Failed for GetSensorOrientation()");

   return 0;
}

extern "C" JNIEXPORT void JNICALL
Java_com_sample_view_ViewActivity_onPreviewSurfaceCreated(
    JNIEnv *env, jobject instance, jobject surface) {

  ASSERT(cameraObj_, "NativeObject should not be null Pointer");

    surface_ = env->NewGlobalRef(surface);
    mnativewindow_ = ANativeWindow_fromSurface(env, surface_);

    yuvProcess_->SetPresentRotation(90);

  ASSERT(yuvImageReader_ != nullptr, "AImageReader is null");
  ANativeWindow *nativeWindow;
  media_status_t status = AImageReader_getWindow(yuvImageReader_, &nativeWindow);
  ASSERT(status == AMEDIA_OK, "Could not get ANativeWindow");

  cameraObj_->CreateSession(nativeWindow);
  cameraObj_->StartPreview(true);
}

extern "C" JNIEXPORT void JNICALL
Java_com_sample_view_ViewActivity_onPreviewSurfaceDestroyed(
    JNIEnv *env, jobject instance, jobject surface) {

  ASSERT(cameraObj_, "NativeObject should not be null Pointer");

  jclass cls = env->FindClass("android/view/Surface");
  jmethodID toString =
      env->GetMethodID(cls, "toString", "()Ljava/lang/String;");

  jstring destroyObjStr =
      reinterpret_cast<jstring>(env->CallObjectMethod(surface, toString));
  const char *destroyObjName = env->GetStringUTFChars(destroyObjStr, nullptr);

  jstring appObjStr = reinterpret_cast<jstring>(
      env->CallObjectMethod(surface_, toString));
  const char *appObjName = env->GetStringUTFChars(appObjStr, nullptr);

  ASSERT(!strcmp(destroyObjName, appObjName), "object Name MisMatch");

  env->ReleaseStringUTFChars(destroyObjStr, destroyObjName);
  env->ReleaseStringUTFChars(appObjStr, appObjName);

  env->DeleteGlobalRef(surface_);
  surface_ = nullptr;
  mnativewindow_ = nullptr;

  cameraObj_->StartPreview(false);
}

extern "C" JNIEXPORT void JNICALL
Java_com_sample_view_ViewActivity_startORsavePic(
        JNIEnv *env, jobject instance, jdoubleArray cal_intrinsic) {
    //save_pic = true;

    jdouble intrinsic[11];
    intrinsic[10] = 0.;

    jsize len = env->GetArrayLength(cal_intrinsic);
    ASSERT(len == 10 || len == 11, "cal_intrinsic length error!");
    if(len == 10) {
        env->GetDoubleArrayRegion(cal_intrinsic, 0, 10, intrinsic);
    }
    else {
        env->GetDoubleArrayRegion(cal_intrinsic, 0, 11, intrinsic);
    }

    svo_ = new SvoSystem(intrinsic);
    svo_->call_back_ = &visualize;
    svo_->set_init_pose(init_Pose);

    start_svo = true;
}

extern "C" JNIEXPORT void JNICALL
Java_com_sample_view_ViewActivity_createGLRender(
        JNIEnv *env, jobject instance, jobject assetManager) {
    AAssetManager *nativeAssetManager = AAssetManager_fromJava(env, assetManager);
    glRender_ = new GLRenderer(nativeAssetManager);

//    sensorManager_ = new sensor_manager();
//    sensorManager_->init();
}

extern "C" JNIEXPORT void JNICALL
Java_com_sample_view_ViewActivity_destroyGLRender(
        JNIEnv *env, jobject instance) {
    delete glRender_;
    glRender_ = nullptr;

    env->DeleteGlobalRef(surface_gl_);
    surface_gl_ = nullptr;
}

extern "C" JNIEXPORT void JNICALL
Java_com_sample_view_ViewActivity_startGLRender(
        JNIEnv *env, jobject instance) {

//    sensor_manager* sensor = sensorManager_;
//    glRender_->update_data_ = [sensor](const float &a, float &x, float &y, float &z){
//        sensor->getFilterdata(a, x, y,z );
//    };

    float gl_pos[16];
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++)
            gl_pos[j*4 + i] = init_Pose[i*4 + j];
    }
    glRender_->setPose(gl_pos);

    glRender_->StartRenderThread();
}

extern "C" JNIEXPORT void JNICALL
Java_com_sample_view_ViewActivity_stopGLRender(
        JNIEnv *env, jobject instance) {
    glRender_->StopRenderThread();
}

extern "C" JNIEXPORT void JNICALL
Java_com_sample_view_ViewActivity_setGLSurface(
        JNIEnv *env, jobject instance, jobject surface) {

    if (surface_gl_)
        env->DeleteGlobalRef(surface_gl_);

    surface_gl_ = env->NewGlobalRef(surface);
    ANativeWindow* window  = ANativeWindow_fromSurface(env, surface_gl_);
    glRender_->SetWindow(window);
}