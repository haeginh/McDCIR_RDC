#include "OCR_ImageProcessor.hh"

ImageProcessor::ImageProcessor(const Config &_config)
	: debugWindow(false), debugBin(false), debugContour(false), //debugPower(false), debugOCR(false),
	  config(_config)//, powerOn(false), key(0)
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
	cv::Mat imgBin = imgGray>imgGray2;

	// find and isolate counter digits
	std::vector<cv::Rect> bBox_raw =  FindCounterDigits(imgBin);
	std::vector<cv::Rect> bBox;
	FindAlignedBoxes(bBox_raw.begin(), bBox_raw.end(), bBox,1);

	std::vector<cv::Mat> digits;
	for(auto roi:bBox){
		digits.push_back(imgBin(roi));
	}
	return digits;
	// if (debugWindow)
	// 	ShowImage();

	// return powerOn;
}

std::vector<cv::Mat> ImageProcessor::Process(cv::Mat &img, std::vector<cv::Rect>& bBox, cv::Mat &imgBin)
{
	//subtract background
	cv::Mat diffBG = abs(img - bgColor);
	cv::Mat diffCHAR = abs(img - charColor);

	// convert to gray and generate binary image
	cv::Mat imgGray, imgGray2;
	cv::cvtColor(diffBG, imgGray, cv::COLOR_BGR2GRAY);
	cv::cvtColor(diffCHAR, imgGray2, cv::COLOR_BGR2GRAY);
	imgBin = imgGray>imgGray2;

	// find and isolate counter digits
	std::vector<cv::Rect> bBox_raw =  FindCounterDigits(imgBin);
	std::vector<cv::Mat> digits;
	if(bBox_raw.size()==0) return digits;

	//FindAlignedBoxes(bBox_raw.begin(), bBox_raw.end(), bBox,1);
	bBox = bBox_raw;
	for(auto roi:bBox){
		digits.push_back(imgBin(roi));
	}
	return digits;
}

void ImageProcessor::FindAlignedBoxes(std::vector<cv::Rect>::const_iterator begin,
									  std::vector<cv::Rect>::const_iterator end, std::vector<cv::Rect> &result, int addMargin=0)
{
	cv::Point2i margin(addMargin, addMargin);
	result.clear();
	std::vector<cv::Rect>::const_iterator it = begin;

	cv::Rect range = *it++;
	result.push_back(cv::Rect(range.tl()-margin,range.br()+margin));

	for (; it != end; ++it)
	{
		int range_b = range.y + range.height;
		int iter_b = it->y + it->height;
		if(iter_b<range.y || it->y>range_b)	continue;

		result.push_back(cv::Rect(it->tl()-margin, it->br()+margin));
		range.y = it->y < range.y ? it->y : range.y;
		int bottom = iter_b > range_b ? iter_b : range_b;
		range.height = bottom - range.y; 

		// if (abs(start.y + start.height - it->y - it->height ) < config.getDigitYAlignment())
		// {
		// 	result.push_back(*it);
		// }
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
