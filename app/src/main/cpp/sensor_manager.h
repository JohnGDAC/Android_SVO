

#ifndef CAMERA_NDK_SENSOR_MANAGER_H
#define CAMERA_NDK_SENSOR_MANAGER_H

#include <android/asset_manager_jni.h>
#include <android/log.h>
#include <android/sensor.h>
#include <dlfcn.h>
#include <jni.h>

#include <cassert>
#include <cstdint>
#include <string>


class sensor_manager {
public:
    const int LOOPER_ID_USER = 3;
    const int SENSOR_REFRESH_RATE_HZ = 100;
    const int32_t SENSOR_REFRESH_PERIOD_US =
            int32_t(1000000 / SENSOR_REFRESH_RATE_HZ);

    ASensorManager *sensorManager;
    const ASensor *accelerometer;
    ASensorEventQueue *accelerometerEventQueue;
    ALooper *looper;

    void init();
    void pause();
    void resume();

    void getFilterdata(const float &a, float &x, float &y, float &z)
    {
        ALooper_pollAll(0, NULL, NULL, NULL);
        ASensorEvent event;
        while (ASensorEventQueue_getEvents(accelerometerEventQueue, &event, 1) >
               0) {
            x = a * event.acceleration.x + (1.0f - a) * x;
            y = a * event.acceleration.y + (1.0f - a) * y;
            z = a * event.acceleration.z + (1.0f - a) * z;
        }
    }
};


#endif //CAMERA_NDK_SENSOR_MANAGER_H
