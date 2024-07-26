

#ifndef CAMERA_NDK_SVO_SYSTEM_H
#define CAMERA_NDK_SVO_SYSTEM_H

#include <list>
#include <thread>
#include <functional>
#include <opencv2/opencv.hpp>
#include <svo/frame_handler_mono.h>

class SvoSystem {
public:
    explicit SvoSystem(double intrinsic[]);

    void main_loop();

    void set_init_pose(double m[]) {
        SE3 pose;
        pose.fromMatrix(m);
        vo_->init_pose_ = std::make_shared<SE3>(pose);
    }

    void add_img(cv::Mat img, double tm) {
        {
            std::lock_guard<std::mutex> lock(queue_img_mutex_);
            queue_img_.emplace_back(img, tm);
        }
        queue_img_con_.notify_one();
    }

    std::function<void(cv::Mat, double [])> call_back_;
private:
    bool running_;
    std::mutex queue_img_mutex_;
    std::condition_variable queue_img_con_;

    svo::FrameHandlerMono* vo_;
    vk::AbstractCamera* cam_;
    std::thread svo_th_;
    std::list<std::pair<cv::Mat, double>> queue_img_;
};


#endif //CAMERA_NDK_SVO_SYSTEM_H
