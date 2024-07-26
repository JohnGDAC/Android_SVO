# README
This is a project that transplants the SVO(Semi-Direct Visual Odometry) to the Android NDK framework for showing a SLAM AR demo.

![demo](demo/svodemo.gif)

**The effect is not good in the platform of Smartphones and it is very easy to lost localization**. Maybe it dues to that my phone's performance is not enough good, or the light intensity and intrinsic of phone's camera are changed arbitrarily. If you are interested, hope you can make some improvement.

The project is not perfect. Before to use on the phone, you should make revisions. At First, revise some code to regulate the *exposure time* ,*sensitivity* and *lens focus distance* of camera:
```
// in camera_manager.cpp

74    exposureRange_.min_ = val.data.i64[0];
74    if (exposureRange_.min_ < kMinExposureTime) {
76      exposureRange_.min_ = kMinExposureTime;
77    }
78    exposureRange_.max_ = val.data.i64[1];
79    if (exposureRange_.max_ > kMaxExposureTime) {
80      exposureRange_.max_ = kMaxExposureTime;
81    }
82    exposureTime_ = exposureRange_.value(10);

....

95    sensitivityRange_.min_ = val.data.i32[0];
96    sensitivityRange_.max_ = val.data.i32[1];

97    sensitivity_ = sensitivityRange_.value(10);


....

293    float focus = 0.5;
294    CALL_REQUEST(setEntry_float(requests_[PREVIEW_REQUEST_IDX].request_,
295                                ACAMERA_LENS_FOCUS_DISTANCE, 1, &focus));
296    focus = 15;
297    CALL_REQUEST(setEntry_float(requests_[PREVIEW_REQUEST_IDX].request_,
298                                ACAMERA_LENS_FOCAL_LENGTH, 1, &focus));
```
Since the program will choose the compatible resolution, as the code shows:
```
// in camera_manager.cpp

173    bool NDKCamera::MatchCaptureSizeRequest(int32_t requestWidth,
174                                            int32_t requestHeight,
175                                            ImageFormat* resCap) {

......

212      if (foundIt) {
213        compatibleCameraRes_.width = foundRes.org_width();
214        compatibleCameraRes_.height = foundRes.org_height();
215        if (resCap) {
216          resCap->width = maxJPG.org_width();
217          resCap->height = maxJPG.org_height();
218        }
219        LOGI("find the compatible camera resolution, taking %d x %d",
220              foundRes.org_width(), foundRes.org_height());
221      } 
  
........  
}
```
So you should revise some code(comment code) as follow to take some photos and then calibrate the camera:
```
// in ViewActivity.java

79    /* if(!is_load_cal_) {
80      Intent intent = new Intent(Intent.ACTION_GET_CONTENT);
...                    
85      intent.setType("*/*");//无类型限制
86      intent.addCategory(Intent.CATEGORY_OPENABLE);
87      startActivityForResult(intent, 1);
88      return;
89    } */
90    startORsavePic(cal_intrinsic_);
91    //button_.setEnabled(false);



// in android_main.cpp

290    extern "C" JNIEXPORT void JNICALL
291    Java_com_sample_view_ViewActivity_startORsavePic(
292            JNIEnv *env, jobject instance, jdoubleArray cal_intrinsic) {
293        save_pic = true;
294
295        /* jdouble intrinsic[11];


311        start_svo = true; */
}
```
As the function `drawFrame` in `android_main.cpp` show, the photos will be saved in `"/sdcard/DCIM/Camera/"`. After calibrate the camera, save the calibraton result in a *json* file with the form as follow:
```
{h: xx, w: xx, 
fx: xx, fy: xx, 
cx: xx, cy: xx,
k1: xx, k2: xx, p1: xx, p2: xx, k3: xx}
```
Then recovery the code and start!  