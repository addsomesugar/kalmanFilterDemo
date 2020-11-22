#include <iostream>
#include <opencv2/opencv.hpp>

#include "kalman/LinearizedKalmanFilter.h"
#include "Self_SystemModel.h"
#include "Self_Measurement.h"

cv::Mat image = cv::Mat::zeros(1000, 1000, CV_8UC3);
int lastX=0;
int lastY=0;
int p_lastX=0;
int p_lastY=0;
bool drawing = false;
KalmanExamples::Robot1::State<float> s_x;
KalmanExamples::Robot1::PositionMeasurement<float> p;
std::shared_ptr<Kalman::LinearizedKalmanFilter<KalmanExamples::Robot1::State<float>>> kf;
KalmanExamples::Robot1::VelocitySystemModel<float> sys;
KalmanExamples::Robot1::PositionMeasurementModel<float> pm;
void mouse_callback(int e, int x, int y, int flag, void* param) {
    switch (e) {
        case cv::EVENT_MOUSEMOVE:
            if(drawing) {
                cv::line(image, cv::Point(lastX, lastY), cv::Point(x, y), cv::Scalar(0, 0, 255), 2);
                lastX = x;
                lastY = y;
                kf->predict(sys);
                p << x, y;
                auto xx = kf->update(pm, p);
                cv::line(image, cv::Point(p_lastX, p_lastY), cv::Point(xx[0], xx[1]), cv::Scalar(0, 255, 0));
                p_lastX = xx[0];
                p_lastY = xx[1];
                cv::imshow("image", image);
            }
            break;
        case cv::EVENT_LBUTTONDOWN:
            drawing = true;
            lastX = x;
            lastY = y;
            s_x << x, y, 0, 0;
            p_lastX = x;
            p_lastY = y;
            kf->init(s_x);
            break;
        case cv::EVENT_LBUTTONUP:
            drawing = false;
            break;
    }
}

int main() {
    cv::namedWindow("image");
    s_x.setZero();
    p.setZero();

    kf = std::make_shared<Kalman::LinearizedKalmanFilter<KalmanExamples::Robot1::State<float>>>();
    cv::setMouseCallback("image", mouse_callback);

    cv::imshow("image", image);
    cv::waitKey();
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
