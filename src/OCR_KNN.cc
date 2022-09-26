#include "OCR_KNN.hh"

KNearestOcr::KNearestOcr()
	: pModel()
{
}

KNearestOcr::~KNearestOcr()
{
}

// Learn a single digit.
int KNearestOcr::learn(const cv::Mat &img)
{
	cv::imshow("Learn", img);
	int key = cv::waitKey(0) & 255;
	if (key >= 176 && key <= 185)
	{
		key -= 128; // numeric keypad
	}
	if ((key >= '0' && key <= '9') || key == '.')
	{
		responses.push_back(cv::Mat(1, 1, CV_32F, (float)key - '0'));
		samples.push_back(prepareSample(img));
		std::cout << char(key) << std::flush;
	}
	return key;
}

// Learn a vector of digits.
int KNearestOcr::learn(const std::vector<cv::Mat> &images)
{
	int key = 0;
	for (std::vector<cv::Mat>::const_iterator it = images.begin();
		 it < images.end() && key != 's' && key != 'q'; ++it)
	{
		key = learn(*it);
	}
	return key;
}

bool KNearestOcr::hasTrainingData()
{
	return !samples.empty() && !responses.empty();
}

// Save training data to file.
void KNearestOcr::saveTrainingData(std::string fileN)
{
	cv::FileStorage fs(fileN, cv::FileStorage::WRITE);
	fs << "samples" << samples;
	fs << "responses" << responses;
	fs.release();

	printTrainingStatus();
}

void KNearestOcr::printTrainingStatus()
{
	std::map<int, int> counter;
	std::cout << "[SUMMARY]" << std::endl;
	for (int i = 0; i < responses.rows; i++)
		counter[(int)responses.at<float>(i, 0)]++;

	for (auto iter : counter)
	{
		char character = iter.first;
		std::cout << character << ": " << iter.second << std::endl;
	}
}

// Load training data from file and init model.
bool KNearestOcr::loadTrainingData(std::string fileN = "training.yml")
{
	cv::FileStorage fs(fileN, cv::FileStorage::READ);
	//	cv::FileStorage fs("lletter.yml", cv::FileStorage::READ);
	if (fs.isOpened())
	{
		fs["samples"] >> samples;
		fs["responses"] >> responses;
		fs.release();

		cv::Ptr<cv::ml::TrainData> trainData = cv::ml::TrainData::create(samples, cv::ml::ROW_SAMPLE, responses);
		pModel->train(trainData);
		trainData.release();
	}
	else
		return false;
	return true;
}

// Recognize a single digit.
char KNearestOcr::recognize(const cv::Mat &img)
{
	int k_idx(3);

	if (pModel.empty())
	{
		throw std::runtime_error("Model is not initialized");
	}

	cv::Mat results, neighborResponses, dists;
	float result = pModel->findNearest(prepareSample(img), k_idx, results, neighborResponses, dists);
	if (((float *)dists.data)[0] > ocrThreshold)
		throw -1;

	return (char)result;
}

// Recognize a vector of digits.
bool KNearestOcr::recognize(const std::vector<cv::Mat> &images, std::string &result)
{
	if(pModel->empty()) return false;
	if(images.size()==0) return false;
	cv::Mat sampleVec;
	for (cv::Mat img : images)
    	sampleVec.push_back(prepareSample(img));  

	cv::Mat results, neighborResponses, dists;
	pModel->findNearest(sampleVec, 1, results, neighborResponses, dists);
	if(cv::countNonZero(dists>ocrThreshold)>0) return false;

	result.clear();
	float *data = (float*)results.data;
	for (int i=0;i<images.size();i++)
	{
		char l = data[i];
		if(l==' ') result += "ON";
		else if(l=='r') result += "ra";
		else if(l==107) continue;//result += "+";
		else result += l;
	}

	return true;
}

// Prepare an image of a digit to work as a sample for the model.
cv::Mat KNearestOcr::prepareSample(const cv::Mat &img)
{
	cv::Mat roi, sample;
	cv::resize(img, roi, cv::Size(10, 10));
	roi.reshape(1, 1).convertTo(sample, CV_32F);

	return sample;
}

// Initialize the model.
void KNearestOcr::initModel()
{
	pModel = cv::ml::KNearest::create();
	pModel->setIsClassifier(true);
	// load persistent model
}

void KNearestOcr::RecogAndLearn(const std::vector<cv::Mat> &digits)
{
	// if(pModel->empty()) return;
	cv::Mat sampleVec, sampleVec2;
	for (cv::Mat img : digits)
		sampleVec.push_back(prepareSample(img));

	cv::Mat results, results2;
	if (samples.cols > 0)
	{
		cv::Mat neighborResponses, dists;
		pModel->findNearest(sampleVec, 1, results, neighborResponses, dists);

		std::string resultStr;
		float *resultData = (float *)results.data;
		for (int i = 0; i < results.rows; i++)
		{
			resultStr.push_back((char)resultData[i]);
		}
		std::cout << "\rrecognized result: " << resultStr << std::endl;

		cv::Mat addChk = dists.col(0) > 0;
		if (cv::countNonZero(addChk) == 0)
			return;

		std::cout << "add " << cv::countNonZero(addChk) << " data" << std::endl;
		cv::Mat overTH = dists.col(0) > ocrThreshold;
		if (cv::countNonZero(overTH) > 0)
		{
			std::cout << cv::countNonZero(overTH) << "over threshold detected!" << std::endl;
			int key = cv::waitKey(0);
			if (key == 'r')
			{
				resultStr.clear();
				std::cout << "Type correct answer: " << std::flush;
				std::cin >> resultStr;
				if (sampleVec.rows != resultStr.length())
				{
					std::cout << resultStr.length() << " inputs for " << sampleVec.rows << " letters!" << std::endl;
					return;
				}
				std::vector<char> vectorData(resultStr.begin(), resultStr.begin() + sampleVec.rows);
				cv::Mat resultVec(vectorData, true);
				resultVec.convertTo(results, CV_32F);
			}
			else if (key == 'q')
			{
				std::cout << "skip this data" << std::endl;
				return;
			}
		}
		char *addChkData = (char *)addChk.data;
		for (int i = 0; i < addChk.rows; i++)
		{
			if (addChkData[i] == 0)
				continue;
			sampleVec2.push_back(sampleVec.row(i));
			results2.push_back(results.row(i));
		}

		cv::vconcat(samples, sampleVec2, samples);
		cv::vconcat(responses, results2, responses);
	}
	else
	{
		// pModel = cv::ml::KNearest::create();
		// pModel->setIsClassifier(true);
		std::cout << "Type the answer: ";
		std::string resultStr;
		std::cin >> resultStr;
		std::vector<char> vectorData(resultStr.begin(), resultStr.begin() + sampleVec.rows);
		cv::Mat resultVec(vectorData, true);
		resultVec.convertTo(responses, CV_32F);
		sampleVec.copyTo(samples);
	}

	cv::Ptr<cv::ml::TrainData> trainData = cv::ml::TrainData::create(samples, cv::ml::ROW_SAMPLE, responses);
	pModel->train(trainData);
	trainData.release();
}

bool KNearestOcr::RecogAndLearn(const cv::Mat &digit)
{
	// if(pModel->empty()) return false;
	cv::Mat img(digit);
	cv::Mat sampleVec;
	sampleVec = prepareSample(digit);

	if (samples.cols > 0)
	{
		cv::Mat neighborResponses, dists, results;
		pModel->findNearest(sampleVec, 1, results, neighborResponses, dists);
		// std::cout<<dists<<std::endl;
		float dist = dists.at<float>(0, 0);
		if (dist == 0)
			return true;

		float *resultData = (float *)results.data;
		char ans = (char)resultData[0];
		;
		//std::cout<<results.type()<<std::endl;
		//std::cout<<resultData[0]<<std::endl;
		//std::cout<<ans<<std::endl;
		std::string ansStr;
		ansStr += ans;

		if (dist > ocrThreshold)
		{
			cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
			cv::resize(img, img, cv::Size((int)(img.cols * 5), (int)(img.rows * 5)));
			cv::putText(img, ansStr, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255., 0., 0.), 2);
			cv::imshow("a digit", img);
			int key = cv::waitKey(0);
			if (key == 'q')
			{
				std::cout << "skip this data" << std::endl;
				return true;
			}
			else if (key == 27) //esc
				return false;
			else if (key != 13) //enter
			{
				std::cout << "add new data: " << (char)key << std::endl;
				resultData[0] = key;
			}
			else
				std::cout << "add new data: " << ans << std::endl;
		}
		else
			std::cout << "add new data: " << ans << std::endl;
		cv::vconcat(samples, sampleVec, samples);
		cv::vconcat(responses, results, responses);
	}
	else
	{
		// pModel = cv::ml::KNearest::create();
		// pModel->setIsClassifier(true);
		// cv::imshow("a digit", digit);
		cv::resize(img, img, cv::Size((int)(img.cols * 5), (int)(img.rows * 5)));
		cv::imshow("a digit", img);

		int key = cv::waitKey(0);
		if (key == 'q')
		{
			std::cout << "skip this data" << std::endl;
			return false;
		}
		std::cout << "add new data: " << (char)key << std::endl;
		sampleVec.copyTo(samples);
		cv::Mat results(cv::Size(1, 1), CV_32F);
		results = key;
		results.copyTo(responses);
	}

	cv::Ptr<cv::ml::TrainData> trainData = cv::ml::TrainData::create(samples, cv::ml::ROW_SAMPLE, responses);
	pModel->train(trainData);
	trainData.release();
	return true;
}