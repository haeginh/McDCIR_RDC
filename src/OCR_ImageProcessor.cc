#include "OCR_ImageProcessor.hh"

ImageProcessor::ImageProcessor(const Config &_config)
	: debugWindow(false), debugBin(false), debugContour(false), //debugPower(false), debugOCR(false),
	  config(_config)											//, powerOn(false), key(0)
{
}

ImageProcessor::~ImageProcessor() {}

std::vector<cv::Mat> ImageProcessor::Process(cv::Mat &img)
{
	//subtract background
	cv::Mat diffBG = abs(img - bgColor);
	cv::Mat diffCHAR = abs(img - charColor);

	// convert to gray and generate binary image
	cv::Mat imgGray, imgGray2;
	cv::cvtColor(diffBG, imgGray, cv::COLOR_BGR2GRAY);
	cv::cvtColor(diffCHAR, imgGray2, cv::COLOR_BGR2GRAY);
	cv::Mat imgBin = imgGray > imgGray2;

	// find and isolate counter digits
	std::vector<cv::Rect> bBox_raw = FindCounterDigits(imgBin);
	std::vector<cv::Rect> bBox;
	FindAlignedBoxes(bBox_raw.begin(), bBox_raw.end(), bBox, 1);

	std::vector<cv::Mat> digits;
	for (auto roi : bBox)
	{
		digits.push_back(imgBin(roi));
	}
	return digits;
	// if (debugWindow)
	// 	ShowImage();

	// return powerOn;
}

std::vector<cv::Mat> ImageProcessor::Process(cv::Mat &img, std::vector<cv::Rect> &bBox, cv::Mat &imgBin)
{
	//subtract background
	cv::Mat diffBG = abs(img - bgColor);
	cv::Mat diffCHAR = abs(img - charColor);

	// convert to gray and generate binary image
	cv::Mat imgGray, imgGray2;
	cv::cvtColor(diffBG, imgGray, cv::COLOR_BGR2GRAY);
	cv::cvtColor(diffCHAR, imgGray2, cv::COLOR_BGR2GRAY);
	imgBin = imgGray > imgGray2;

	// find and isolate counter digits
	std::vector<cv::Rect> bBox_raw = FindCounterDigits(imgBin);
	std::vector<cv::Mat> digits;
	if (bBox_raw.size() == 0)
		return digits;

	FindAlignedBoxes(bBox_raw.begin(), bBox_raw.end(), bBox, 1);
	//bBox = bBox_raw;
	for (auto &roi : bBox)
	{
		if(roi.x<0) roi.x=0;
		if(roi.y<0) roi.y=0;
		if(roi.br().x>imgBin.cols-1) roi.width=(imgBin.cols-1) - roi.x;
		if(roi.br().y>imgBin.rows-1) roi.height=(imgBin.rows-1) - roi.y;

		digits.push_back(imgBin(roi));
	}
	return digits;
}

void ImageProcessor::FindAlignedBoxes(std::vector<cv::Rect>::const_iterator begin,
									  std::vector<cv::Rect>::const_iterator end, std::vector<cv::Rect> &result, int addMargin = 0)
{
	cv::Point2i margin(addMargin, addMargin);
	result.clear();
	std::vector<cv::Rect>::const_iterator it = begin;

	result.push_back(cv::Rect(it->tl() - margin, it->br() + margin));
	int rangeTop = result.back().tl().y;
	int rangeBottom = result.back().br().y;

	for (auto prev = it++; it != end; prev = it++)
	{
		if (it->y > rangeBottom || it->br().y < rangeTop)
			continue;

		//i, j (Vertically separated letters)
		cv::Rect merged;
		if (prev->br().y <= it->y)
			merged = cv::Rect(prev->tl(), it->br());
		else if (it->br().y <= prev->y)
			merged = cv::Rect(cv::Point2i(prev->x, it->y), cv::Point2i(it->br().x, prev->br().y));
		
		if (merged.width > 0 && merged.width < it->width + prev->width)
			result.back() = cv::Rect(merged.tl() - margin, merged.br() + margin);
		else
		 	result.push_back(cv::Rect(it->tl() - margin, it->br() + margin));
		
		rangeTop = it->y < rangeTop ? it->y:rangeTop;
		rangeBottom = it->br().y > rangeBottom ? it->br().y : rangeBottom;
	}
}

void ImageProcessor::FilterContours(std::vector<std::vector<cv::Point>> &contours,
									std::vector<cv::Rect> &boundingBoxes, std::vector<std::vector<cv::Point>> &filteredContours)
{
	// filter contours by bounding rect size
	for (size_t i = 0; i < contours.size(); i++)
	{
		cv::Rect bounds = cv::boundingRect(contours[i]);
		if (bounds.height > config.getDigitMinHeight() && bounds.height < config.getDigitMaxHeight())
		{
			boundingBoxes.push_back(bounds);
			filteredContours.push_back(contours[i]);
		}
	}
}

std::vector<cv::Rect> ImageProcessor::FindCounterDigits(cv::Mat &imgBin)
{
	// find contours in whole image
	std::vector<std::vector<cv::Point>> contours, filteredContours;
	cv::findContours(imgBin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	std::vector<cv::Rect> boundingBoxes;
	FilterContours(contours, boundingBoxes, filteredContours);

	// sort bounding boxes from left to right
	std::sort(boundingBoxes.begin(), boundingBoxes.end(), sortRectByX());

	return boundingBoxes;
}
