#ifndef INCLUDE_OCR_MAIN_HH
#define INCLUDE_OCR_MAIN_HH

#include <iostream>
#include <functional>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include "OCR_ImageProcessor.hh"
#include "OCR_KNN.hh"

using namespace cv;
using namespace std;

enum SOURCE{ VIDEO, CAPTUREBOARD };

class OcrMain
{
    public:
    OcrMain(string _dir);
    ~OcrMain(){}

    bool SetSource(SOURCE opt, string videoName="");
    bool Initialize(string configName);
    bool Render(float* data);
    void RenderForSetting();
    void RenderForLearning();

    private:
    string dir;
    cv::VideoCapture cap;
    SOURCE source;
    ImageProcessor proc;
    KNearestOcr ocr;
    vector<cv::Rect> roiBox;
    vector<string> roiNames;
    int resize;
    bool showName;
    bool stop;

    //settings

    bool learning;
};

#endif