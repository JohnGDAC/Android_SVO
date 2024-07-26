#include "sensor_manager.h"

static ASensorManager *AcquireASensorManagerInstance(void) {
    const char *kPackageName = "com.android.accelerometergraph";

    static ASensorManager* pf_getinstanceforpackage = nullptr;
    static ASensorManager* pf_getinstance = nullptr;
    if(pf_getinstanceforpackage) {
        return pf_getinstanceforpackage;
    } else if(pf_getinstance){
        return pf_getinstance;
    }

    void *androidHandle = dlopen("libandroid.so", RTLD_NOW);

    typedef ASensorManager *(*PF_GETINSTANCEFORPACKAGE)(const char *name);
    PF_GETINSTANCEFORPACKAGE getInstanceForPackageFunc =
            (PF_GETINSTANCEFORPACKAGE)dlsym(androidHandle,
                                            "ASensorManager_getInstanceForPackage");
    if (getInstanceForPackageFunc) {
        pf_getinstanceforpackage = getInstanceForPackageFunc(kPackageName);
        return pf_getinstanceforpackage;
    }

    typedef ASensorManager *(*PF_GETINSTANCE)();
    PF_GETINSTANCE getInstanceFunc =
            (PF_GETINSTANCE)dlsym(androidHandle, "ASensorManager_getInstance");
    // by all means at this point, ASensorManager_getInstance should be available
    assert(getInstanceFunc);
    pf_getinstance = getInstanceFunc();
    return pf_getinstance;
}


void sensor_manager::init() {

    sensorManager = AcquireASensorManagerInstance();
    assert(sensorManager != NULL);

    accelerometer = ASensorManager_getDefaultSensor(sensorManager,
                                                    ASENSOR_TYPE_ACCELEROMETER);
    assert(accelerometer != NULL);

    looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    assert(looper != NULL);

    accelerometerEventQueue = ASensorManager_createEventQueue(
            sensorManager, looper, LOOPER_ID_USER, NULL, NULL);
    assert(accelerometerEventQueue != NULL);

    auto status =
            ASensorEventQueue_enableSensor(accelerometerEventQueue, accelerometer);
    assert(status >= 0);


    status = ASensorEventQueue_setEventRate(
            accelerometerEventQueue, accelerometer, SENSOR_REFRESH_PERIOD_US);
    assert(status >= 0);
    (void)status;  // to silent unused compiler warning

}

void sensor_manager::pause() {
    ASensorEventQueue_disableSensor(accelerometerEventQueue, accelerometer);
}

void sensor_manager::resume() {
    ASensorEventQueue_enableSensor(accelerometerEventQueue, accelerometer);
    auto status = ASensorEventQueue_setEventRate(
            accelerometerEventQueue, accelerometer, SENSOR_REFRESH_PERIOD_US);
    assert(status >= 0);
}