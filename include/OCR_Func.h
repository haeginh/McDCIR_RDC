#ifndef INCLUDE_OCR_FUNC_H_
#define INCLUDE_OCR_FUNC_H_

#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include "OCR_ImageProcessor.hh"
#include "OCR_KNN.hh"

int DELAY = 1;

//cv::Rect cropRect(0,0,0,0);
cv::Point P1(0,0), P2(0,0);
cv::Scalar rgb(0,0,0);
std::vector<cv::Rect> blackBox;
std::vector<cv::Rect> blackPoint;
bool clicked=false;
bool rclicked=false;

void onMouseCropImage(int event, int x, int y, int f, void *param){
	cv::Mat* img = (cv::Mat*)param;
    switch (event) {
    case cv::EVENT_LBUTTONDOWN:
        clicked = true;
        P1.x = x;
        P1.y = y;
        P2.x = x;
        P2.y = y;
        break;
    case cv::EVENT_LBUTTONUP:
        P2.x=x;
        P2.y=y;
        clicked = false;
        break;
    case cv::EVENT_MOUSEMOVE:
        if(clicked){
        P2.x=x;
        P2.y=y;
        }
        if(rclicked){
        rgb = img->at<cv::Vec3b>(y,x);
		cv::imshow("color",cv::Mat(100,100,CV_8UC3,rgb));
        }
        break;
    case cv::EVENT_RBUTTONDOWN:
        rclicked = true;
        rgb = img->at<cv::Vec3b>(y,x);
		cv::imshow("color",cv::Mat(100,100,CV_8UC3,rgb));
        break;
    case cv::EVENT_RBUTTONUP:
        rclicked = false;
        break;
    default:
        break;
    }
}

// Adapted from cv_timer in cv_utilities
class Timer
{
public:
    Timer() : start_(0), time_(0) {}

    void start()
    {
        start_ = cv::getTickCount();
    }

    void stop()
    {
        CV_Assert(start_ != 0);
        int64 end = cv::getTickCount();
        time_ += end - start_;
        start_ = 0;
    }

    double time()
    {
        double ret = time_ / cv::getTickFrequency();
        time_ = 0;
        return ret;
    }

private:
    int64 start_, time_;
};

#endif /* INCLUDE_OCR_FUNC_H_ */
