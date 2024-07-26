//
// Created by xingkong on 2017/9/21.
//
#include "GLRenderer.h"
#include <assert.h>

GLRenderer::GLRenderer(AAssetManager *assetManager) {
    pthread_mutex_init(&renderPara_._mutex, 0);
    pthread_mutex_init(&pos_mutex_, 0);

    AAsset *vertexShaderAsset =
            AAssetManager_open(assetManager, "shader.glslv", AASSET_MODE_BUFFER);
    assert(vertexShaderAsset != NULL);
    const void *vertexShaderBuf = AAsset_getBuffer(vertexShaderAsset);
    assert(vertexShaderBuf != NULL);
    off_t vertexShaderLength = AAsset_getLength(vertexShaderAsset);
    vertexShaderSource_ =
            std::string((const char *) vertexShaderBuf, (size_t) vertexShaderLength);
    AAsset_close(vertexShaderAsset);

    AAsset *fragmentShaderAsset =
            AAssetManager_open(assetManager, "shader.glslf", AASSET_MODE_BUFFER);
    assert(fragmentShaderAsset != NULL);
    const void *fragmentShaderBuf = AAsset_getBuffer(fragmentShaderAsset);
    assert(fragmentShaderBuf != NULL);
    off_t fragmentShaderLength = AAsset_getLength(fragmentShaderAsset);
    fragmentShaderSource_ = std::string((const char *) fragmentShaderBuf,
                                       (size_t) fragmentShaderLength);
    AAsset_close(fragmentShaderAsset);

    //generateXPos();
}

GLRenderer::~GLRenderer() {
    running = 0;
    Inited = 0;
    pthread_join(renderPara_._threadId,NULL);
    DestroyRender();
    pthread_mutex_destroy(&renderPara_._mutex);
    pthread_mutex_destroy(&pos_mutex_);
}

void GLRenderer::SetWindow(ANativeWindow* window) {
    // notify render thread that window has changed
    if (window) {
        pthread_mutex_lock(&renderPara_._mutex);
        renderPara_._window  = window;
        renderPara_._msg = renderPara_.MSG_WINDOW_SET;
        pthread_mutex_unlock(&renderPara_._mutex);
    } else {
        ANativeWindow_release(renderPara_._window);
    }
    return;
}


bool GLRenderer::InitRender(){
    /*
     * Here specify the attributes of the desired configuration.
     * Below, we select an EGLConfig with at least 8 bits per color
     * component compatible with on-screen windows
     */
    const EGLint attribs[] = {
            EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
            EGL_BLUE_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_RED_SIZE, 8,
            EGL_NONE
    };
    EGLint attribList[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE }; // OpenGL 2.0
    EGLint w, h, format;
    EGLint numConfigs;
    EGLConfig config;
    EGLSurface surface;
    EGLContext context;

    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);

    eglInitialize(display, 0, 0);
    /* Here, the application chooses the configuration it desires.
     * find the best match if possible, otherwise use the very first one
     */
    eglChooseConfig(display, attribs, nullptr,0, &numConfigs);
    std::unique_ptr<EGLConfig[]> supportedConfigs(new EGLConfig[numConfigs]);
//    assert(supportedConfigs);
    eglChooseConfig(display, attribs, supportedConfigs.get(), numConfigs, &numConfigs);
//    assert(numConfigs);
    auto i = 0;
    for (; i < numConfigs; i++) {
        auto& cfg = supportedConfigs[i];
        EGLint r, g, b, d;
        if (eglGetConfigAttrib(display, cfg, EGL_RED_SIZE, &r)   &&
            eglGetConfigAttrib(display, cfg, EGL_GREEN_SIZE, &g) &&
            eglGetConfigAttrib(display, cfg, EGL_BLUE_SIZE, &b)  &&
            eglGetConfigAttrib(display, cfg, EGL_DEPTH_SIZE, &d) &&
            r == 8 && g == 8 && b == 8 && d == 0 ) {

            config = supportedConfigs[i];
            break;
        }
    }
    if (i == numConfigs) {
        config = supportedConfigs[0];
    }

    /* EGL_NATIVE_VISUAL_ID is an attribute of the EGLConfig that is
     * guaranteed to be accepted by ANativeWindow_setBuffersGeometry().
     * As soon as we picked a EGLConfig, we can safely reconfigure the
     * ANativeWindow buffers to match, using EGL_NATIVE_VISUAL_ID. */
    eglGetConfigAttrib(display, config, EGL_NATIVE_VISUAL_ID, &format);
    surface = eglCreateWindowSurface(display, config,renderPara_._window, NULL);
    context = eglCreateContext(display, config, NULL, attribList);

    if (eglMakeCurrent(display, surface, surface, context) == EGL_FALSE) {
        return -1;
    }

    eglQuerySurface(display, surface, EGL_WIDTH, &w);
    eglQuerySurface(display, surface, EGL_HEIGHT, &h);

    renderPara_._display = display;
    renderPara_._surface = surface;
    renderPara_._context = context;
    renderPara_._width = w;
    renderPara_._height = h;
    glEnable(GL_CULL_FACE);
//    glShadeModel(GL_SMOOTH);
    glDisable(GL_DEPTH_TEST);

    SurfaceCreate();
    SurfaceChange(w,h);

    window_seted = 1;

    return true;
}

void GLRenderer::DestroyRender() {

    eglMakeCurrent(renderPara_._display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(renderPara_._display, renderPara_._context);
    eglDestroySurface(renderPara_._display, renderPara_._surface);
    eglTerminate(renderPara_._display);

    renderPara_._display = EGL_NO_DISPLAY;
    renderPara_._surface = EGL_NO_SURFACE;
    renderPara_._context = EGL_NO_CONTEXT;

    window_seted = 0;
    return;
}

void GLRenderer::StartRenderThread(){
    if(Inited == 0){
        pthread_create(&renderPara_._threadId, 0, RenderThread, this);
        Inited = 1;
    }
    running = 1;
}

void GLRenderer::StopRenderThread() {

//    // send message to render thread to stop rendering
//    pthread_mutex_lock(&renderPara._mutex);
//    renderPara._msg = renderPara.MSG_RENDER_LOOP_EXIT;
//    pthread_mutex_unlock(&renderPara._mutex);
//    pthread_join(renderPara._threadId, 0);
//    LOGE("Renderer thread stopped");
    running = 0;

    return;
}

void* GLRenderer::RenderThread(void *args){
    GLRenderer *render= (GLRenderer*)args;
    render->RenderLoop();
    pthread_exit(0);
    return 0;
}

void GLRenderer::RenderLoop() {

    while (running) {

        // process incoming messages
        if (renderPara_._msg == renderPara_.MSG_WINDOW_SET) {
            InitRender();
        }
        else if (renderPara_._msg == renderPara_.MSG_RENDER_LOOP_EXIT) {
            DestroyRender();
        }
        renderPara_._msg = renderPara_.MSG_NONE;

        if (window_seted == 0) { //如果没有初始化窗口，就不能启动
            usleep(16000);
            continue;
        }

        if (renderPara_._display) {
            pthread_mutex_lock(&renderPara_._mutex);

//            update_data_(SENSOR_FILTER_ALPHA, sensorDataFilter.x, sensorDataFilter.y, sensorDataFilter.z);
//            sensorData[sensorDataIndex] = sensorDataFilter;
//            sensorData[SENSOR_HISTORY_LENGTH + sensorDataIndex] = sensorDataFilter;
//            sensorDataIndex = (sensorDataIndex + 1) % SENSOR_HISTORY_LENGTH;

            DrawFrame();
            if (!eglSwapBuffers(renderPara_._display, renderPara_._surface)) {
//                    LOGE("GLrenderS::eglSwapBuffers() returned error %d", eglGetError());
            }
            pthread_mutex_unlock(&renderPara_._mutex);
        } else {
            usleep(16000);  //如果没有初始化好EGL，就等下一帧再渲染
        }
    }

}

void GLRenderer::SurfaceCreate() {

    LOGI("GL_VERSION: %s", glGetString(GL_VERSION));
    LOGI("GL_VENDOR: %s", glGetString(GL_VENDOR));
    LOGI("GL_RENDERER: %s", glGetString(GL_RENDERER));
    LOGI("GL_EXTENSIONS: %s", glGetString(GL_EXTENSIONS));

    shaderProgram = createProgram(vertexShaderSource_, fragmentShaderSource_);
    assert(shaderProgram != 0);

    GLint getPositionLocationResult =
            glGetAttribLocation(shaderProgram, "vPoints");
    assert(getPositionLocationResult != -1);
    vPositionHandle = (GLuint)getPositionLocationResult;

    GLint getSensorValueLocationResult =
        glGetUniformLocation(shaderProgram, "vViewPose");
    assert(getSensorValueLocationResult != -1);
    vViewPoseHandle = (GLuint)getSensorValueLocationResult;

    GLint getFragColorLocationResult =
            glGetAttribLocation(shaderProgram, "uFragColor");
    assert(getFragColorLocationResult != -1);
    uFragColorHandle = (GLuint)getFragColorLocationResult;

}

void GLRenderer::SurfaceChange(int width, int height) {
    glViewport(0, 0, width, height);
}

GLuint GLRenderer::createProgram(const std::string &pVertexSource,
                     const std::string &pFragmentSource) {
    GLuint vertexShader = loadShader(GL_VERTEX_SHADER, pVertexSource);
    GLuint pixelShader = loadShader(GL_FRAGMENT_SHADER, pFragmentSource);
    GLuint program = glCreateProgram();
    assert(program != 0);
    glAttachShader(program, vertexShader);
    glAttachShader(program, pixelShader);
    glLinkProgram(program);
    GLint programLinked = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &programLinked);
    assert(programLinked != 0);
    glDeleteShader(vertexShader);
    glDeleteShader(pixelShader);
    return program;
}

GLuint GLRenderer::loadShader(GLenum shaderType, const std::string &pSource) {
    GLuint shader = glCreateShader(shaderType);
    assert(shader != 0);
    const char *sourceBuf = pSource.c_str();
    glShaderSource(shader, 1, &sourceBuf, NULL);
    glCompileShader(shader);
    GLint shaderCompiled = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &shaderCompiled);
    assert(shaderCompiled != 0);
    return shader;
}

void GLRenderer::DrawFrame() {

    const GLfloat cube_array[][3] = {{0., 0., 0.},{1., 0., 0.},{0., 1., 0.},
                                     {1., 1., 0.},{0., 1., 0.},{1., 0., 0.},

                                     {0., 0., 1.},{1., 0., 1.},{0., 1., 1.},
                                     {1., 1., 1.},{0., 1., 1.},{1., 0., 1.},

                                     {0., 0., 0.},{1., 0., 0.},{0., 0., 1.},
                                     {1., 0., 1.},{0., 0., 1.},{1., 0., 0.},

                                     {0., 1., 0.},{1., 1., 0},{0., 1., 1.},
                                     {1., 1., 1.},{0., 1., 1.},{1., 1., 0.},

                                     {0., 0., 0.},{0., 0., 1.},{0., 1., 0.},
                                     {0., 1., 1.},{0., 1., 0.},{0., 0., 1.},

                                     {1., 0., 0.},{1., 0., 1.},{1., 1., 0.},
                                     {1., 1., 1.},{1., 1., 0.},{1., 0., 1.}};

    const GLfloat cube_color[][3] = {{0., 0., 1.}, {0., 0., 1.}, {0., 0., 1.},
                                     {0., 0., 1.}, {0., 0., 1.}, {0., 0., 1.},

                                     {0., 0., 1.}, {0., 0., 1.}, {0., 0., 1.},
                                     {0., 0., 1.}, {0., 0., 1.}, {0., 0., 1.},

                                     {0., 1., 0.}, {0., 1., 0.}, {0., 1., 0.},
                                     {0., 1., 0.}, {0., 1., 0.}, {0., 1., 0.},

                                     {0., 1., 0.}, {0., 1., 0.}, {0., 1., 0.},
                                     {0., 1., 0.}, {0., 1., 0.}, {0., 1., 0.},

                                     {1., 0., 0.}, {1., 0., 0.}, {1., 0., 0.},
                                     {1., 0., 0.}, {1., 0., 0.}, {1., 0., 0.},

                                     {1., 0., 0.}, {1., 0., 0.}, {1., 0., 0.},
                                     {1., 0., 0.}, {1., 0., 0.}, {1., 0., 0.}};

    GLfloat viewPos[16];
    pthread_mutex_lock(&pos_mutex_);
    memcpy(viewPos, pos_, 16 * sizeof(GLfloat));
    pthread_mutex_unlock(&pos_mutex_);

    glClearColor(1.f, 1.f, 1.f, 1.0f);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    glUseProgram(shaderProgram);

    //glLineWidth(2.0);

    glUniformMatrix4fv(vViewPoseHandle, 1, GL_FALSE, viewPos);

    glEnableVertexAttribArray(vPositionHandle);
    glVertexAttribPointer(vPositionHandle, 3, GL_FLOAT, GL_FALSE,
                                3*sizeof(GLfloat), cube_array[0]);

    glEnableVertexAttribArray(uFragColorHandle);
    glVertexAttribPointer(uFragColorHandle, 3, GL_FLOAT, GL_FALSE,
                          3*sizeof(GLfloat), cube_color[0]);

    //glUniform4f(uFragColorHandle, 1.0f, 0.0f, 0.0f, 1.0f);

    glDrawArrays(GL_TRIANGLES, 0, 36);
}

void GLRenderer::setPose(float pos[]) {
    pthread_mutex_lock(&pos_mutex_);
    memcpy(pos_, pos, 16 * sizeof(GLfloat));
    pthread_mutex_unlock(&pos_mutex_);
}

//void GLRenderer::DrawFrame() {
//    glClearColor(1.f, 1.f, 1.f, 1.0f);
//    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
//
//    glUseProgram(shaderProgram);
//
//    glLineWidth(2.0);
//
//    glEnableVertexAttribArray(vPositionHandle);
//    glVertexAttribPointer(vPositionHandle, 1, GL_FLOAT, GL_FALSE, 0, xPos);
//
//    glEnableVertexAttribArray(vSensorValueHandle);
//
//    glVertexAttribPointer(vSensorValueHandle, 1, GL_FLOAT, GL_FALSE,
//                          sizeof(AccelerometerData),
//                          &sensorData[sensorDataIndex].x);
//    glUniform4f(uFragColorHandle, 1.0f, 0.0f, 0.0f, 1.0f);
//    glDrawArrays(GL_LINE_STRIP, 0, SENSOR_HISTORY_LENGTH);
//
//    glVertexAttribPointer(vSensorValueHandle, 1, GL_FLOAT, GL_FALSE,
//                          sizeof(AccelerometerData),
//                          &sensorData[sensorDataIndex].y);
//    glUniform4f(uFragColorHandle, 0.0f, 1.0f, 0.0f, 1.0f);
//    glDrawArrays(GL_LINE_STRIP, 0, SENSOR_HISTORY_LENGTH);
//
//    glVertexAttribPointer(vSensorValueHandle, 1, GL_FLOAT, GL_FALSE,
//                          sizeof(AccelerometerData),
//                          &sensorData[sensorDataIndex].z);
//    glUniform4f(uFragColorHandle, 0.0f, 0.0f, 1.0f, 1.0f);
//    glDrawArrays(GL_LINE_STRIP, 0, SENSOR_HISTORY_LENGTH);
//
//
//}