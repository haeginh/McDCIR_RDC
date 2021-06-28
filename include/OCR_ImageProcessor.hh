#ifndef INCLUDE_OCR_IMAGEPROCESSOR_HH_
#define INCLUDE_OCR_IMAGEPROCESSOR_HH_

#include <iostream>
#include <fstream>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include "OCR_Config.hh"

class sortRectByX {
public:
	bool operator()(cv::Rect const& a, cv::Rect const& b) const {
		return a.x < b.x;
	}
};

class ImageProcessor {
public:
	ImageProcessor(const Config& config);
	~ImageProcessor();

	std::vector<cv::Mat> Process(cv::Mat& _img);
	std::vector<cv::Mat> Process(cv::Mat& img, std::vector<cv::Rect>& bBox, cv::Mat& imgBin);
	void SetBGcolor(cv::Scalar color) {bgColor = color;}
	void SetCHARcolor(cv::Scalar color) {charColor = color;}

//	int ShowImage();
	//void SetInput(cv::Mat& _img) { img = _img; }
//	bool GetPowerOn() { return powerOn; }

	// const std::vector<cv::Mat>& GetOutputkV()    { return digits_kV;  };
	// const std::vector<cv::Mat>& GetOutputmA()    { return digits_mA;  };
	// const std::vector<cv::Mat>& GetOutputDAP()   { return digits_DAP; };
	// const std::vector<cv::Mat>& GetOutputTime()  { return digits_Time;};
//	void DebugWindow(bool bval = true) { debugWindow = bval; }
	void DebugBinary  (bool bval = true) { debugBin     = bval; }
	void DebugContour (bool bval = true) { debugContour = bval; }
//	void DebugDigits(bool bval = true) { debugDigits = bval; }
	// void DebugPower (bool bval = true) { debugPower  = bval; }
	// void DebugOCR   (bool bval = true) { debugOCR    = bval; }



private:
	void FindAlignedBoxes(std::vector<cv::Rect>::const_iterator begin,
			std::vector<cv::Rect>::const_iterator end, std::vector<cv::Rect>& result, int addMargin);
	void FilterContours(std::vector<std::vector<cv::Point> >& contours,
			std::vector<cv::Rect>& boundingBoxes, std::vector<std::vector<cv::Point> >& filteredContours);
	std::vector<cv::Rect> FindCounterDigits(cv::Mat& imgBin);



	// cv::Mat img;
	// cv::Mat imgGray;
	// cv::Mat imgBin;

	// std::vector<cv::Mat> digits_kV;
	// std::vector<cv::Mat> digits_mA;
	// std::vector<cv::Mat> digits_DAP;
	// std::vector<cv::Mat> digits_Time;

	Config config;

	bool debugWindow;
	bool debugBin;
	bool debugContour;
//	bool debugDigits;
//	bool debugPower;
//	bool debugOCR;

//	bool powerOn;

	cv::Scalar bgColor, charColor;

//	int key;

};

#endif /* INCLUDE_OCR_IMAGEPROCESSOR_HH_ */
