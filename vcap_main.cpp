#include <iostream>
#include <functional>
#include "OCR_Func.h"

using namespace std;

int main(int argc, char **argv)
{
	// Initialize Capture board
	cv::VideoCapture cap(0);
	cv::waitKey(30);
	if (!cap.isOpened())
	{
		cout << "Can't open the capture board" << endl;
		return -1;
	}
	//load config file
	Config config;
	if (argc == 1)
		config.loadConfig("./data/config.yml");
	else if (argc == 2)
		config.loadConfig(string(argv[1]));

	cap.set(cv::CAP_PROP_FPS, config.getCaptureBoardFPS());
	//	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920*0.5);

	//Set the roi boxes
	std::vector<cv::Rect> roiBox = config.getRoiBoxes();
	std::cout << "    >> Select 4 ROI box to OCR, Last ROI box will check the On/Off" << std::endl
			  << "       ";
	std::vector<std::string> roiNames = config.GetRoiNames();
	for (size_t i = 0; i < roiNames.size(); i++)
		std::cout << i << ". " << roiNames[i] << " ";
	std::cout << std::endl;

	double resize_factor(1);
	bool pushROI(false), learning(false);
	int colorCount(0);
	while (1)
	{
		cv::Mat img;
		cap >> img;
		cv::Mat imgResize;

		cv::resize(img, imgResize, cv::Size(floor(img.cols * resize_factor), floor(img.rows * resize_factor)));
		cv::setMouseCallback("CaptureBoard", onMouseCropImage, &imgResize);

		for (cv::Rect box : roiBox)
		{
			box.x *= resize_factor;
			box.y *= resize_factor;
			box.height *= resize_factor;
			box.width *= resize_factor;

			cv::rectangle(imgResize, box, CV_RGB(255, 255, 0), 2);
		}

		if (clicked)
		{
			cv::rectangle(imgResize, P1, P2, CV_RGB(255, 255, 0), 2);
			pushROI = true;
		}
		else if (pushROI)
		{
			if (roiBox.size() == roiNames.size())
			{
				std::cout << "Do you want to reset ROI boxes [y(Y)]? : " << std::flush;
				string ans;
				std::cin >> ans;
				if (ans == "y" || ans == "Y")
				{
					roiBox.clear();
					std::cout << "Reset ROI boxex!" << std::endl;
				}
				pushROI = false;
			}
			else
			{
				std::cout << "Selected " << roiBox.size() << ". " << roiNames[roiBox.size()] << std::endl;
				roiBox.push_back(cv::Rect(P1 / resize_factor, P2 / resize_factor));
				pushROI = false;
			}
		}

		if (learning)
		{
		}
		cv::putText(imgResize, "Resolution: " + std::to_string(img.cols) + "x" + std::to_string(img.rows),
					cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(60000), 1);
		cv::imshow("CaptureBoard", imgResize);

		int key = cv::waitKey(1);
		if (key == 'q')
			break;
		else if (key == 'r')
		{
			std::cout << "Reset ROI boxes!" << std::endl;
			roiBox.clear();
		}
		else if (key == 'b')
		{
			cout << "Set background color : "<< rgb << endl;
			config.setBGcolor(rgb);
		}
		else if (key == 'c')
		{
			cout << "Set charactor color : "<< rgb << endl;
			config.setCHARcolor(rgb);
		}
		else if (key == 's')
		{
			cout << "Enter a new resize factor: " << flush;
			cin >> resize_factor;
		}
		else if (key == 'p')
		{
			config.setRoiBoxes(roiBox);
			cout << "Print configuration file: " << flush;
			{
				string fileN;
				cin >> fileN;
				config.saveConfig(fileN);
			}
		}
		else if (key == 'l')
		{
			learning = true;
			break;
		}
	}

	//Start OCR
	cout << "    Initialize OCR" << endl;
	ImageProcessor proc(config);
	proc.SetBGcolor(config.getBGcolor());
	proc.SetCHARcolor(config.getCHARcolor());
	//proc.DebugBinary();
	proc.DebugContour();
	KNearestOcr ocr;
	ocr.SetOcrThreshold(config.getOcrThreshold());
	if (!ocr.loadTrainingData("excel.yml"))
	{
		cout << "      Failed to load OCR training data" << endl;
		return 1;
	}
	cout << "      OCR training data loaded." << endl;

	std::vector<cv::Rect> smallBox;
	bool firstLoop(true);
	while (1)
	{
		cv::Mat img;
		cap >> img;
		cv::Mat imgCopy, imgResize;
		img.copyTo(imgCopy);

		for (cv::Rect box : roiBox)
			cv::rectangle(imgCopy, box, CV_RGB(255, 255, 0), 2);

		Timer ocr_timer;
		smallBox.clear();
		ocr_timer.start();

		for (size_t i = 0; i < roiBox.size(); i++)
		{
			cv::Mat imgCrop = img(roiBox[i]);
			cv::Mat imgBin;
			std::vector<cv::Rect> bBox;
			auto digits = proc.Process(imgCrop, bBox, imgBin);

			if (learning)
			{
				for (cv::Rect box : bBox)
				{
					cv::rectangle(imgCrop, box, CV_RGB(255, 0, 0), 1);
				}
				cv::imshow("binary", imgBin);
				cv::imshow("crop", imgCrop);
				if (firstLoop)
					cv::waitKey(0);
				ocr.RecogAndLearn(digits);
			}
			else
			{
				string ocrResult = ocr.recognize(digits);
				for (cv::Rect box : bBox)
				{
					box.x = box.x + roiBox[i].x;
					box.y = box.y + roiBox[i].y;
					smallBox.push_back(box);
				}
				cv::putText(imgCopy,  roiNames[i] + ": " + ocrResult,
							roiBox[i].tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(60000), 1);

//				cout << roiNames[i] << ": " << ocrResult << " ";
			}
		}
		if (!learning)
		{
			ocr_timer.stop();
			cout  << "Time -> " << ocr_timer.time() << endl;

			for (cv::Rect box : smallBox)
				cv::rectangle(imgCopy, box, CV_RGB(255, 0, 0), 1);
		}
		cv::imshow("CaptureBoard", imgCopy);
		int key = cv::waitKey(1);
		firstLoop = false;

		if (learning && key == 'p')
		{
			std::cout << "Training file name: " << std::flush;
			std::string fileN;
			std::cin >> fileN;
			ocr.saveTrainingData(fileN);
		}
		if (learning && key == 's')
		{
			ocr.printTrainingStatus();
			cv::waitKey(0);
		}
	}

	return 0;
}
