#ifndef INCLUDE_OCR_KNN_HH_
#define INCLUDE_OCR_KNN_HH_

#include "OCR_Config.hh"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
#include <vector>
#include <list>
#include <string>

class KNearestOcr {
public:
	KNearestOcr();
	virtual ~KNearestOcr();

	int learn(const cv::Mat& img);
	int learn(const std::vector<cv::Mat>& images);
	bool hasTrainingData();
	void saveTrainingData(std::string fileN);
    void printTrainingStatus();
	bool loadTrainingData(std::string fileN);

	char recognize(const cv::Mat& img);
	std::string recognize(const std::vector<cv::Mat>& images);

private:
	cv::Mat prepareSample(const cv::Mat& img);
	void initModel();

	cv::Mat samples;
	cv::Mat responses;
	cv::Ptr<cv::ml::KNearest> pModel;
	
    //specific training
public:
    void RecogAndLearn(const std::vector<cv::Mat> &digits);
    void SetOcrThreshold(int n) {ocrThreshold = n;}
private:
    int ocrThreshold;


};

#endif /* INCLUDE_OCR_KNN_HH_ */