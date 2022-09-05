#include "OCR_ImageProcessor.hh"

ImageProcessor::ImageProcessor()
	: debugWindow(false), debugBin(false), debugContour(false), //debugPower(false), debugOCR(false),
	  binThresh(50)											    //, powerOn(false), key(0)
{
}

ImageProcessor::~ImageProcessor() {}

std::vector<cv::Mat> ImageProcessor::Process(cv::Mat &img, int &top, int &bottom)
{
	//subtract background
	cv::Mat diffBG = abs(img - bgColor);
	//cv::Mat diffCHAR = abs(img - charColor);

	// convert to gray and generate binary image
	cv::Mat imgGray;//, imgGray2;
	cv::cvtColor(diffBG, imgGray, cv::COLOR_BGR2GRAY);
	// cv::cvtColor(diffCHAR, imgGray2, cv::COLOR_BGR2GRAY);
	// cv::Mat imgBin = imgGray > imgGray2;
	std::vector<cv::Mat> digits;
	double min, max;
	cv::minMaxLoc(imgGray, &min, &max);
	if(max<binThresh)  return digits;
	cv::Mat imgBin = imgGray > max-binThresh;

	// find and isolate counter digits
	std::vector<cv::Rect> bBox_raw = FindCounterDigits(imgBin);
	if (bBox_raw.size() == 0)
		return digits;
	std::vector<cv::Rect> bBox;
	FindAlignedBoxes(bBox_raw.begin(), bBox_raw.end(), bBox, top, bottom, 0);
	for (auto roi : bBox)
	{
		digits.push_back(imgBin(roi));
	}
	return digits;
	// if (debugWindow)
	// 	ShowImage();

	// return powerOn;
}

std::vector<cv::Mat> ImageProcessor::Process(cv::Mat &img, std::vector<cv::Rect> &bBox, cv::Mat &imgBin, int &top, int &bottom, bool fitting)
{
	//subtract background
	cv::Mat diffBG = abs(img - bgColor);
	// cv::Mat diffCHAR = abs(img - charColor);
	// convert to gray and generate binary image
	cv::Mat imgGray;//, imgGray2;
	cv::cvtColor(diffBG, imgGray, cv::COLOR_BGR2GRAY);
	// cv::cvtColor(diffCHAR, imgGray2, cv::COLOR_BGR2GRAY);
	// imgBin = imgGray > imgGray2;
	std::vector<cv::Mat> digits;
	double min, max;
	cv::minMaxLoc(imgGray, &min, &max);
	if(max<binThresh)  return digits;
	imgBin = imgGray > max-binThresh;

	// find and isolate counter digits
	std::vector<cv::Rect> bBox_raw = FindCounterDigits(imgBin);
	if (bBox_raw.size() == 0)
		return digits;

	FindAlignedBoxes(bBox_raw.begin(), bBox_raw.end(), bBox, top, bottom, 0);
	// bBox = bBox_raw;

	for (auto &roi : bBox)
	{
		// if(roi.x<0) roi.x=0;
		// if(roi.y<0) roi.y=0;
		// if(roi.br().x>imgBin.cols-1) roi.width=(imgBin.cols-1) - roi.x;
		// if(roi.br().y>imgBin.rows-1) roi.height=(imgBin.rows-1) - roi.y;
		if(fitting) {roi.y = 0; roi.height = img.rows;}
		digits.push_back(imgBin(roi));
	}

	return digits;
}
void ImageProcessor::FindAlignedBoxes(std::vector<cv::Rect>::const_iterator begin,
									  std::vector<cv::Rect>::const_iterator end, std::vector<cv::Rect> &result, int &top, int &bottom, int addMargin = 0)
{
	cv::Point2i margin(addMargin, addMargin);
	result.clear();
	std::vector<cv::Rect>::const_iterator it = begin;

	result.push_back(cv::Rect(it->tl() - margin, it->br() + margin));
	top = result.back().tl().y;
	bottom = result.back().br().y;

	for (auto prev = it++; it != end; prev = it++)
	{
		if (it->y > bottom || it->br().y < top)
			continue;
		if (it->height < config.getDigitMinHeight()) continue;
		if (it->height > config.getDigitMaxHeight()) continue;
		if (it->width < config.getDigitMinWidth()) continue;

	 	result.push_back(cv::Rect(it->tl() - margin, it->br() + margin));
		
		top = it->y < top ? it->y:top;
		bottom = it->br().y > bottom ? it->br().y : bottom;
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

	// box merging
	// |   [     ]
	// [    ]    |
	std::vector<cv::Rect> boundingBoxes1;
	int prev=0; 
	int top(boundingBoxes[0].y), bottom(boundingBoxes[0].br().y), right(boundingBoxes[0].br().x);
	for(int i=1;i<boundingBoxes.size();i++)
	{
		if(boundingBoxes[i].x<right)
		{
			top = std::min(top, boundingBoxes[i].y);
			bottom = std::max(bottom, boundingBoxes[i].br().y);
			right = std::max(right, boundingBoxes[i].br().x);
			continue;
		}
		boundingBoxes1.push_back(cv::Rect(cv::Point(boundingBoxes[prev].x, top), cv::Point(right, bottom)));
		prev = i;
		top = boundingBoxes[i].y;
		bottom = boundingBoxes[i].br().y;
		right = boundingBoxes[i].br().x;
	}
	boundingBoxes1.push_back(cv::Rect(cv::Point(boundingBoxes[prev].x, top), cv::Point(right, bottom)));

	return boundingBoxes1;
}
