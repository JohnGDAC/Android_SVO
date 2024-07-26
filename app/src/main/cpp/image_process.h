
#ifndef CAMERA_IMAGE_READER_H
#define CAMERA_IMAGE_READER_H
#include <media/NdkImageReader.h>

#include <functional>

#include <opencv2/opencv.hpp>

class ImageProcess {
public:
    /**
     * Ctor and Dtor()
     */
    explicit ImageProcess(int32_t rotation);

    /**
     * DisplayImage()
     *   Present camera image to the given display buffer. Avaliable image is
     * converted
     *   to display buffer format. Supported display format:
     *      WINDOW_FORMAT_RGBX_8888
     *      WINDOW_FORMAT_RGBA_8888
     *   @param buf {@link ANativeWindow_Buffer} for image to display to.
     *   @param image a {@link AImage} instance, source of image conversion.
     *            it will be deleted via {@link AImage_delete}
     *   @return true on success, false on failure
     */
    bool DisplayImage(ANativeWindow_Buffer* buf, AImage* image);
    /**
     * Configure the rotation angle necessary to apply to
     * Camera image when presenting: all rotations should be accumulated:
     *    CameraSensorOrientation + Android Device Native Orientation +
     *    Human Rotation (rotated degree related to Phone native orientation
     */
    void SetPresentRotation(int32_t angle) {
        presentRotation_ = angle;
    }

    cv::Mat GetCVImage(AImage* image);

    void DisplayImage(ANativeWindow_Buffer* buf, cv::Mat image);
private:
    int32_t presentRotation_;

    void PresentImage(ANativeWindow_Buffer* buf, AImage* image);
    void PresentImage90(ANativeWindow_Buffer* buf, AImage* image);
    void PresentImage180(ANativeWindow_Buffer* buf, AImage* image);
    void PresentImage270(ANativeWindow_Buffer* buf, AImage* image);

    void WriteFile(AImage* image);
};

#endif  // CAMERA_IMAGE_READER_H
