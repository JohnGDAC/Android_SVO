
#include "svo_system.h"
#include <svo/pinhole_camera.h>
#include <svo/feature.h>

//static const int image_width = 640;
//static const int image_height = 480;
//
//static const double fx = 6.0970550296798035e+02;
//static const double fy = 6.0909579671294716e+02;
//static const double cx = 3.1916667152289227e+02;
//static const double cy = 2.3558360480225772e+02;
//
//static const double k1 = 9.2615504465028850e-02;
//static const double k2 = -1.8082438825995681e-01;
//static const double p1 = -6.5484100374765971e-04;
//static const double p2 = -3.5829351558557421e-04;

SvoSystem::SvoSystem(double intrinsic[]) {
    SVO_INFO_STREAM("camera intrinsic %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
                    intrinsic[0], intrinsic[1], intrinsic[2], intrinsic[3],
                    intrinsic[4], intrinsic[5], intrinsic[6], intrinsic[7],
                    intrinsic[8], intrinsic[9], intrinsic[10]);
    cam_ = new vk::PinholeCamera(intrinsic[0], intrinsic[1], intrinsic[2],
                                 intrinsic[3], intrinsic[4], intrinsic[5],
                                 intrinsic[6], intrinsic[7], intrinsic[8],
                                 intrinsic[9], intrinsic[10]);
    vo_ = new svo::FrameHandlerMono(cam_);
    running_ = true;
    svo_th_ = std::thread(&SvoSystem::main_loop, this);
}

void SvoSystem::main_loop()
{
    vo_->start();
    while (running_)
    {
        cv::Mat img;
        double tm;
        {
            std::unique_lock<std::mutex> lock(queue_img_mutex_);
            queue_img_con_.wait(lock, [this](){ return !queue_img_.empty();});

            img = queue_img_.front().first;
            tm = queue_img_.front().second;
            queue_img_.pop_front();
        }

        cv::Mat out_mat;
        cv::cvtColor(img, out_mat, cv::COLOR_RGBA2GRAY);
        vo_->addImage(out_mat, tm);
        //static_cast<vk::PinholeCamera*>(cam_)->undistortImage(img, out_mat);

        if(call_back_)
        {
            if(vo_->lastFrame()) {
                double TMatrix_f_w[16];
                vo_->lastFrame()->T_f_w_.getMatrix(TMatrix_f_w);
                TMatrix_f_w[12] = 0.;
                TMatrix_f_w[13] = 0.;
                TMatrix_f_w[14] = 0.;
                TMatrix_f_w[15] = 1.;

                for(auto ft : vo_->lastFrame()->fts_)
                    cv::circle(img, cv::Point2f(ft->px.x(), ft->px.y()), 3,
                               cv::Scalar(255, 0,0));

                call_back_(img, TMatrix_f_w);
            }
        }

    }
}