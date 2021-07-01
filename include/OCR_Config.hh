#ifndef INCLUDE_OCR_CONFIG_HH_
#define INCLUDE_OCR_CONFIG_HH_

#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <map>

class Config {
public:
    Config();
    void saveConfig(std::string fileN);
    void loadConfig(std::string fileN);


    int getDigitMinHeight() const {
        return _digitMinHeight;
    }

    int getDigitMaxHeight() const {
        return _digitMaxHeight;
    }

    int getCaptureBoardWidth() const {
        return _captureBoardWidth;
    }

    int getCaptureBoardHeight() const {
        return _captureBoardHeight;
    }

    int getCaptureBoardFPS() const {
        return _captureBoardFPS;
    }

    std::string getTrainingDataFilename() const {
        return _trainingDataFilename;
    }

    float getOcrThreshold() const {
        return _ocrThreshold;
    }

    int getRotationDegrees() const {
        return _rotationDegrees;
    }

    cv::Scalar getBGcolor() {return _bgColor;}
    void setBGcolor(cv::Scalar rgb) {_bgColor = rgb;}

    cv::Scalar getCHARcolor() {return _charColor;}
    void setCHARcolor(cv::Scalar rgb) {_charColor = rgb;}

    int getRoiID(std::string name) {
    	if(roiID.find(name)==roiID.end()) return -1;
        return roiID[name];
    }

    std::vector<std::string> GetRoiNames() const{
        return roiNames;
    }

    std::vector<cv::Rect> getRoiBoxes() {return _roiBox;}
    void setRoiBoxes(std::vector<cv::Rect> roiBox) {_roiBox = roiBox;}

private:
    int _rotationDegrees;
    float _ocrThreshold;
    int _digitMinHeight;
    int _digitMaxHeight;
    int _captureBoardWidth;
    int _captureBoardHeight;
    int _captureBoardFPS;
    std::string _trainingDataFilename;
    cv::Scalar _bgColor, _charColor;
    std::vector<cv::Rect> _roiBox;
    std::map<std::string, int> roiID;
    std::vector<std::string> roiNames;
};

#endif /* INCLUDE_OCR_CONFIG_HH_ */
