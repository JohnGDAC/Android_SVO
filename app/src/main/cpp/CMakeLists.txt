

cmake_minimum_required(VERSION 3.4.1)

set(CMAKE_VERBOSE_MAKEFILE on)
#set(COMMON_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../common)

# now build app's shared lib
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror")

set(OpenCV_STATIC ON)
set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../jniLibs/OpenCV-android-sdk/sdk/native/jni)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE SVO_CPP "${CMAKE_CURRENT_SOURCE_DIR}/svo/*.cpp")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/svo/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../jniLibs)
add_library(SVO_LIB SHARED
        ${SVO_CPP})
message(${SVO_CPP})
target_link_libraries(SVO_LIB ${OpenCV_LIBS})

# Export ANativeActivity_onCreate(),
# Refer to: https://github.com/android-ndk/ndk/issues/381.
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS} -u ANativeActivity_onCreate")
add_library(system_main SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/android_main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_listeners.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/image_process.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GLRenderer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/svo_system.cpp)

# add lib dependencies
add_dependencies(system_main SVO_LIB)
target_link_libraries(system_main
        dl
        android
        log
        m
        camera2ndk
        mediandk
        EGL
        GLESv2
        ${OpenCV_LIBS}
        SVO_LIB)
