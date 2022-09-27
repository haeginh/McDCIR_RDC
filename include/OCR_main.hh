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
    void SetRecord(bool chk, double factor=0.5){
        recording = chk;
        recordSize = cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH)*factor, cap.get(cv::CAP_PROP_FRAME_HEIGHT)*factor);
        if(recording){
            time_t now = chrono::system_clock::to_time_t(chrono::system_clock::now());
            recorder.open(string(ctime(&now))+"_ocr.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 
		    20 , recordSize, true);
            if(!recorder.isOpened())
            {
                cout<<"error in recorder!"<<endl;
            }
        }
    }

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
    bool recording;
    VideoWriter recorder;
    cv::Size recordSize;;
};

#endif