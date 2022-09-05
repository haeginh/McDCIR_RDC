#include "OCR_Config.hh"
#include <sstream>
#include <iostream>

Config::Config() : _digitMinHeight(1), _digitMaxHeight(50), _captureBoardWidth(1280.), _captureBoardHeight(720.),
                   _captureBoardFPS(10), _ocrThreshold(255 * 255 * 5), _bgColor(255., 255., 255., 0.),// _charColor(0., 0., 0., 0.),
                   _boxFileName("box.yml"), _trainingDataFilename("traing.yml")
{
}

void Config::saveConfig(std::string fileN)
{
    cv::FileStorage fs(fileN, cv::FileStorage::WRITE);
    fs << "digitMinWidth" << _digitMinWidth;
    fs << "digitMinHeight" << _digitMinHeight;
    fs << "digitMaxHeight" << _digitMaxHeight;
    fs << "captureboardWidth" << _captureBoardWidth;
    fs << "captureboardHeight" << _captureBoardHeight;
    fs << "captureboardFPS" << _captureBoardFPS;
    fs << "ocrThreshold" << _ocrThreshold;
    fs << "binThreshold" << _binThreshold;
    fs << "bgColor" << _bgColor;
    // fs << "charColor" << _charColor;
    fs << "roiNames" << _roiNames;
    fs << "boxFilename" << _boxFileName;
    fs << "trainingDataFilename" << _trainingDataFilename;
    fs.release();
}

bool Config::loadConfig(std::string fileN)
{
    cv::FileStorage fs(fileN, cv::FileStorage::READ);
    if(!fs.isOpened()) return false;

    fs["digitMinWidth"] >> _digitMinWidth;
    fs["digitMinHeight"] >> _digitMinHeight;
    fs["digitMaxHeight"] >> _digitMaxHeight;
    fs["captureboardWidth"] >> _captureBoardWidth;
    fs["captureboardHeight"] >> _captureBoardHeight;
    fs["captureboardFPS"] >> _captureBoardFPS;
    fs["ocrThreshold"] >> _ocrThreshold;
    fs["binThreshold"] >> _binThreshold;
    fs["bgColor"] >> _bgColor;
    // fs["charColor"] >> _charColor;
    _roiNames.clear();
    fs["roiNames"] >> _roiNames;
    std::stringstream ss(_roiNames);
    std::string str;
    while (ss >> str)
        roiNames.push_back(str);
    fs["boxFilename"] >> _boxFileName;
    fs["trainingDataFilename"] >> _trainingDataFilename;
    fs.release();
    return true;
}

void Config::saveBoxConfig(std::string fileN)
{
    cv::FileStorage fs(fileN, cv::FileStorage::WRITE);
    for (size_t i = 0; i < _roiBox.size(); i++)
        fs << roiNames[i] << cv::Scalar(_roiBox[i].x, _roiBox[i].y, _roiBox[i].width, _roiBox[i].height);
    fs.release();
}

bool Config::loadBoxConfig(std::string fileN)
{
    cv::FileStorage fs(fileN, cv::FileStorage::READ);
    if(!fs.isOpened()) return false;
    cv::Scalar tmp;
    _roiBox.clear();
    for (std::string name:roiNames)
    {
        fs[name] >> tmp;
        _roiBox.push_back(cv::Rect(tmp[0],tmp[1],tmp[2],tmp[3]));
    }
    
    fs.release();
    return true;
}