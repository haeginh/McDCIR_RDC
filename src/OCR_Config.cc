#include "OCR_Config.hh"
#include <sstream>

Config::Config() :
        _rotationDegrees(0), _ocrThreshold(255*255*5), _digitMinHeight(20), _digitMaxHeight(90),
        _captureBoardFPS(10),  _trainingDataFilename("trainctr.yml")
{}

void Config::saveConfig(std::string fileN) {
    cv::FileStorage fs(fileN, cv::FileStorage::WRITE);
    fs << "digitMinHeight" << _digitMinHeight;
    fs << "digitMaxHeight" << _digitMaxHeight;
    fs << "captureboardFPS" << _captureBoardFPS;
    fs << "ocrThreshold" << _ocrThreshold; _ocrThreshold *=255*255;
    fs << "trainingDataFilename" << _trainingDataFilename;
    fs << "bgColor" << _bgColor;
    fs << "charColor" << _charColor;
    std::stringstream ss;
    for(std::string name:roiNames) ss<<" "<<name;
    fs << "roiNames" << ss.str();
    for(size_t i=0;i<_roiBox.size();i++) fs<<roiNames[i]<<cv::Scalar(_roiBox[i].x,_roiBox[i].y,_roiBox[i].width,_roiBox[i].height);
    fs.release();
}

#include <iostream>
void Config::loadConfig(std::string fileN) {
    cv::FileStorage fs(fileN, cv::FileStorage::READ);
    if (fs.isOpened()) {
        fs["digitMinHeight"] >> _digitMinHeight;
        fs["digitMaxHeight"] >> _digitMaxHeight;
        fs["captureboardFPS"] >> _captureBoardFPS;
        fs["ocrThreshold"] >> (int)_ocrThreshold/(255*255);
        fs["trainingDataFilename"] >> _trainingDataFilename;
        fs["bgColor"] >> _bgColor;
        fs["charColor"] >> _charColor;
        std::string str;
        fs["roiNames"] >> str;
        std::stringstream ss(str);
        while(ss>>str) roiNames.push_back(str);
        for(size_t i=0;i<roiNames.size();i++){
            roiID[roiNames[i]] = i;
            cv::Scalar boxInfo;
            fs[roiNames[i]]>>boxInfo;
            if(boxInfo[2]==0 || boxInfo[3]==0 ) {
                std::cerr<<"Check ROI boxex of config file"<<std::endl;
                exit(100);
            }
            _roiBox.push_back(cv::Rect(boxInfo[0],boxInfo[1],boxInfo[2],boxInfo[3]));
        }

        fs.release();
    } else {
        // no config file - create an initial one with default values
    }
}
