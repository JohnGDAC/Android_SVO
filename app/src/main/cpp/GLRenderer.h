//
// Created by xingkong on 2017/8/14.
//

#ifndef NATIVEGL_DEMO2_GLINTERFACE_H
#define NATIVEGL_DEMO2_GLINTERFACE_H

#include <GLES2/gl2.h>
#include <EGL/egl.h>
#include <pthread.h>
#include <cstdlib>
#include <math.h>
#include <android/native_window.h>
#include <android/asset_manager.h>
#include <unistd.h>
#include <sys/time.h>
#include <functional>
#include <string>
#include <memory>

#include "native_debug.h"

struct RenderPara{

    enum RenderThreadMessage {
        MSG_NONE = 0,
        MSG_WINDOW_SET,
        MSG_RENDER_LOOP_EXIT
    };

    pthread_t _threadId;
    pthread_mutex_t _mutex;
    enum RenderThreadMessage _msg;

    // android window, supported by NDK r5 and newer
    ANativeWindow* _window;

    EGLDisplay _display;
    EGLSurface _surface;
    EGLContext _context;

    EGLint _width;
    EGLint _height;
};

class GLRenderer {
private:
    RenderPara renderPara_;

//    int start;
    bool Inited = 0;
    bool running = 0;
    bool window_seted = 0;

    std::string vertexShaderSource_;
    std::string fragmentShaderSource_;

    GLuint shaderProgram;
    GLuint vPositionHandle;
    GLuint vViewPoseHandle;
    // GLuint vSensorValueHandle;
    GLuint uFragColorHandle;

    pthread_mutex_t pos_mutex_;
    GLfloat pos_[16] = {1., 0., 0., 0.,
                        0., 1., 0., 0.,
                        0., 0., 1., 0.,
                        0., 0., 0., 1.,};

public:
    /*
     * 下面的函数为GLRender类内置函数，用户不需要实现
     * 其中声明为virtual的函数用户程序可覆盖实现
     * */
    GLRenderer(AAssetManager *assetManager);
    ~GLRenderer();
    bool InitRender();  //初始化EGL
    void DestroyRender();   //清除EGL
    static void * RenderThread(void *args); //OpenGL渲染线程入口
    void RenderLoop();  //渲染线程主循环

    /*
     * 将OpenGL渲染的窗口关联到java空间的Surface对象。实现在SurfaceView上的绘图
     * @env:JNI方法传入的JNIEnv对象
     * @surface:Surface对象
     */
    void SetWindow(ANativeWindow* window);

    void StartRenderThread();

    void StopRenderThread();

    void SurfaceCreate();

    void SurfaceChange(int width, int height);

    GLuint createProgram(const std::string &pVertexSource,
                         const std::string &pFragmentSource);

    GLuint loadShader(GLenum shaderType, const std::string &pSource);

    void DrawFrame();

    void setPose(float pos[]);

//    struct AccelerometerData {
//        GLfloat x;
//        GLfloat y;
//        GLfloat z;
//    };
//    AccelerometerData sensorData[SENSOR_HISTORY_LENGTH * 2];
//    AccelerometerData sensorDataFilter;
//    int sensorDataIndex = 0;
//
//    void generateXPos() {
//        for (auto i = 0; i < SENSOR_HISTORY_LENGTH; i++) {
//            float t =
//                    static_cast<float>(i) / static_cast<float>(SENSOR_HISTORY_LENGTH - 1);
//            xPos[i] = -1.f * (1.f - t) + 1.f * t;
//        }
//    }
//    std::function<void(const float &a, float &x, float &y, float &z)> update_data_;
//    static const int SENSOR_HISTORY_LENGTH = 100;
//    const float SENSOR_FILTER_ALPHA = 0.1f;
//    GLfloat xPos[SENSOR_HISTORY_LENGTH];

};



#endif
